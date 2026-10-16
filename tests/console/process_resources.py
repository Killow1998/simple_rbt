"""Sample owned GUI/core and isolated Docker processes, including startup."""
import os
from pathlib import Path
import subprocess
import threading
import time
from observe_session import owned_workers


def stats(pid, container=None):
    path=Path('/proc')/str(pid)
    values=(path/'stat').read_text().split(')',1)[1].split()
    cpu=(int(values[11])+int(values[12]))/os.sysconf('SC_CLK_TCK')
    rss=int((path/'statm').read_text().split()[1])*os.sysconf('SC_PAGESIZE')/1048576
    try:
        rollup=(path/'smaps_rollup').read_text()
    except PermissionError:
        if not container:
            raise
        namespace_pid=next(line.split()[-1] for line in (path/'status').read_text().splitlines() if line.startswith('NSpid:'))
        rollup=subprocess.run(['docker','exec',container,'cat',f'/proc/{namespace_pid}/smaps_rollup'],
                              capture_output=True,text=True,check=True,timeout=5).stdout
    pss=next(float(line.split()[1])/1024 for line in rollup.splitlines() if line.startswith('Pss:'))
    return dict(cpu_seconds=cpu,rss_mib=rss,pss_mib=pss)


class Sampler:
    def __init__(self, gui):
        self.gui=gui
        self.container=None
        self.client=None
        self.phase='startup'
        self.samples=[]
        self.errors=[]
        self.done=threading.Event()
        self.thread=threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        try:
            self.collect()
        except Exception as error:
            self.errors.append(str(error))

    def collect(self):
        start=previous_wall=time.monotonic()
        previous={}
        while not self.done.wait(.5):
            now=time.monotonic()
            processes={'gui':[self.gui], 'core':[pid for pid,_ in owned_workers(self.gui)]}
            if self.container:
                result=subprocess.run(['docker','top',self.container,'-eo','pid,args'],capture_output=True,text=True,timeout=5)
                processes['ros1']=[]
                processes['receiver']=[]
                if result.returncode==0:
                    for line in result.stdout.splitlines()[1:]:
                        fields=line.strip().split(maxsplit=1)
                        if not fields or 'smaps_rollup' in line:
                            continue
                        group='receiver' if 'ros_resource_sink.py' in line else 'ros1'
                        processes[group].append(int(fields[0]))
                if self.client:
                    processes['ros1'].append(self.client)
            groups={}
            for group,pids in processes.items():
                total=dict(cpu_percent_one_core=0.,rss_mib=0.,pss_mib=0.)
                for pid in pids:
                    try:
                        value=stats(pid,self.container if group in ('ros1','receiver') and pid!=self.client else None)
                    except (FileNotFoundError,ProcessLookupError,subprocess.CalledProcessError):
                        continue
                    except Exception as error:
                        self.errors.append(str(error));continue
                    total['cpu_percent_one_core']+=100*max(0,value['cpu_seconds']-previous.get(pid,0))/(now-previous_wall)
                    previous[pid]=value['cpu_seconds']
                    for key in ('rss_mib','pss_mib'):
                        total[key]+=value[key]
                groups[group]=total
            total={key:sum(value[key] for name,value in groups.items() if name!='receiver') for key in ('cpu_percent_one_core','rss_mib','pss_mib')}
            self.samples.append(dict(elapsed_s=now-start,phase=self.phase,interval_s=now-previous_wall,groups=groups,total=total))
            previous_wall=now

    def stop(self):
        self.done.set();self.thread.join(timeout=15)
        if self.thread.is_alive():
            raise RuntimeError('Sampler did not exit')
        if self.errors:
            raise RuntimeError('Resource samples incomplete: '+repr(self.errors[:5]))

    def summary(self):
        result={}
        for phase in sorted({row['phase'] for row in self.samples}):
            rows=[row for row in self.samples if row['phase']==phase]
            result[phase]={}
            for group in ('gui','core','ros1','receiver','total'):
                pairs=[(row['total'] if group=='total' else row['groups'].get(group),row['interval_s']) for row in rows]
                pairs=[(value,interval) for value,interval in pairs if value]
                values=[value for value,interval in pairs]
                if not values:
                    continue
                result[phase][group]=dict(mean_cpu_percent_one_core=sum(v['cpu_percent_one_core']*interval for v,interval in pairs)/sum(interval for v,interval in pairs),
                    peak_cpu_percent_one_core=max(v['cpu_percent_one_core'] for v in values),
                    peak_rss_mib=max(v['rss_mib'] for v in values),peak_pss_mib=max(v['pss_mib'] for v in values),samples=len(values))
        return result
