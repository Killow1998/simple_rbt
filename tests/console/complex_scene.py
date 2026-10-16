"""Deterministic multi-storey resource fixture; not a traversability benchmark."""


def building():
    floors = [dict(id=f"floor-{i}", name=f"Floor {i}", height=3*i) for i in range(4)]
    scene = dict(version=1, id="resource-building", name="Four-floor resource building",
                 floors=floors, objects=[], spawn=dict(floor="floor-0", position=[-16,-11,0], yaw_deg=0))

    def add(name, floor, kind, position, size, **extra):
        obj = dict(id=name, name=name, floor=f"floor-{floor}", kind=kind,
                   position=position, size=size, quaternion=[1,0,0,0], **extra)
        scene["objects"].append(obj)
        return obj

    for level in range(4):
        slab = add(f"slab-{level}",level,"floor",[0,0,-.1],[48,32,.2])
        if level:
            slab["opening"] = dict(center=[-16,5], size=[12,6])
        for sign in (-1,1):
            add(f"wall-x-{level}-{sign}",level,"wall",[sign*24,0,1.5],[.2,32,3])
            add(f"wall-y-{level}-{sign}",level,"wall",[0,sign*16,1.5],[48,.2,3])
        for row,y in enumerate((-6,2,10)):
            for col,x in enumerate((-20,-12,-4,4,12,20)):
                tag=f"{level}-{row}-{col}"
                add("partition-"+tag,level,"wall",[x,y,1.2],[.15,4,2.4])
                add("desk-"+tag,level,"obstacle",[x+2,y, .45],[1.2,.8,.9])
                add("cabinet-"+tag,level,"obstacle",[x+3,y+1, .8],[.6,.6,1.6])
        if level<3:
            add(f"ramp-{level}",level,"ramp",[-22,2,0],[12,2,.2],
                connection=dict(to_floor=f"floor-{level+1}",turn_deg=90,landing_m=2))
            add(f"stairs-{level}",level,"stairs",[-5,-6,0],[4.76,1.2,.2],
                connection=dict(to_floor=f"floor-{level+1}",steps=17,turn_deg=180,landing_m=1.2))
    return scene
