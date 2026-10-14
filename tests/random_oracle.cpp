// Golden binary64 words generated independently by NumPy 1.26.4:
// np.random.default_rng(seed).uniform(0., 1., size=60).view(np.uint64).
// Scaled hashes cover the actual rough-terrain maxima .025/.05/.08/.12 m:
// np.random.default_rng(seed).uniform(0., height, size=60).view(np.uint64).
// FNV-1a is over each binary64 word's bytes, least significant byte first.
// No Python or third-party dependency is needed when running this executable.
#include "simple_rbt/random.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
struct Golden {
  std::uint64_t seed;
  std::array<std::uint64_t, 60> unit_bits;
  std::array<std::uint64_t, 4> scaled_hashes;
};
constexpr std::array<double, 4> heights{.025, .05, .08, .12};
const std::array<Golden, 7> golden{{
  Golden{0ULL, {{
    0x3fe461fd79fb3850ULL, 0x3fd1442f7e20b674ULL, 0x3fa4fa7b529d9bd0ULL, 0x3f90ec9ed84d0bc0ULL, 0x3fea064f4f059bcaULL,
    0x3fed354b2f34c803ULL, 0x3fe3698f6e301db6ULL, 0x3fe758092bff1053ULL, 0x3fe165603cf43b8fULL, 0x3fedec1d00f1ead7ULL,
    0x3fea1b78e9a8e006ULL, 0x3f666f0d1577d200ULL, 0x3feb6fdb17ec9cdeULL, 0x3fa13220e71bcf20ULL, 0x3fe7595660f879faULL,
    0x3fc67be224ec5c90ULL, 0x3feb9f296742db6fULL, 0x3fe153a67b20b50fULL, 0x3fd32e7ac8052544ULL, 0x3fdb0d4eb3dec9b4ULL,
    0x3f9cffd4f59cea80ULL, 0x3fbfd10760015ec8ULL, 0x3fe575c1552043ddULL, 0x3fe4b5c6c750d1dcULL, 0x3fe3b13c1e083bccULL,
    0x3fd88e2c4cf0e646ULL, 0x3fefe924cfaebfdeULL, 0x3fef6300cad9c278ULL, 0x3fe5eff5be6c250eULL, 0x3fe4d08ff8de6738ULL,
    0x3fe607c1701ae3d4ULL, 0x3fd8e416af2d0e5aULL, 0x3fc14ad79f70051cULL, 0x3fe7166eb7328660ULL, 0x3fe0cfb3de3b0e2dULL,
    0x3fd3db00bd5806d2ULL, 0x3fdf17ed305b11c2ULL, 0x3fec76af30d70096ULL, 0x3fede3af3a425594ULL, 0x3fd6e61dd3220188ULL,
    0x3fe249f8ed758714ULL, 0x3fd4998213104b76ULL, 0x3fe304817f37063aULL, 0x3fd5a05667a04864ULL, 0x3fd9104923f0aea4ULL,
    0x3fec7d20a34a8069ULL, 0x3fcd138001a1c85cULL, 0x3fe3f1262ab6da80ULL, 0x3fb58207910c6d88ULL, 0x3feaa50556ec1185ULL,
    0x3fe92fe8ca2cd3bdULL, 0x3fcea3a86ca881f4ULL, 0x3fec0c28a8595583ULL, 0x3fadfca1242899f0ULL, 0x3fd582f121aaf5faULL,
    0x3fc33c5b89c96d50ULL, 0x3fdcd25c34f70ae8ULL, 0x3fe97b7d093ca3c9ULL, 0x3fcd85af145a48a8ULL, 0x3faaa2893588b570ULL,
  }}, {{0x5f7f506fd214114eULL, 0x8ffc611c6f883beeULL, 0x5b79f224c732705bULL, 0x33141f0229f16e7bULL}}},
  Golden{1ULL, {{
    0x3fe060d7be6f245cULL, 0x3fee6a32d7782a55ULL, 0x3fc273d27b04760cULL, 0x3fee5b5615da558dULL, 0x3fd3f50be80ff35cULL,
    0x3fdb17c7d1779e1cULL, 0x3fea7c8a2668bdcdULL, 0x3fda30519310728eULL, 0x3fe1964580549a3fULL, 0x3f9c3874c857cc00ULL,
    0x3fe81cc785dbb2e6ULL, 0x3fe13878535acff3ULL, 0x3fd51a530eb452a6ULL, 0x3fe93aced50e7c5aULL, 0x3fd3678b4907ee46ULL,
    0x3fdd061c0307f682ULL, 0x3fc1284740fd0b40ULL, 0x3fd9cc9a6958882cULL, 0x3fca0ad24273b210ULL, 0x3fd0c9bde4a024feULL,
    0x3fe802fcc620a262ULL, 0x3fd1f2379343f824ULL, 0x3fdf0d5e71dfe33eULL, 0x3fef6232fae3bdf4ULL, 0x3feec5e54e97f8a6ULL,
    0x3fe7317aac82f284ULL, 0x3fe151bafb896174ULL, 0x3fd1b895e27b3210ULL, 0x3fc4903eb9dce978ULL, 0x3fef09a1052a528aULL,
    0x3fe083a2442d79cbULL, 0x3fbda95e684afff8ULL, 0x3fe3f3a0c9ad9651ULL, 0x3fe8da968bee9443ULL, 0x3fe39db9194b9c7aULL,
    0x3fed5a80b758bab1ULL, 0x3fa445847cdac8c0ULL, 0x3fe0ea3407d6fe2dULL, 0x3fdd65c254b7d068ULL, 0x3fafec48b6bc74e0ULL,
    0x3fe485c2a70ea127ULL, 0x3feb48c4a998a46cULL, 0x3fe2f95f7126e000ULL, 0x3fd0a56fc3f37bfeULL, 0x3feae04f362b0744ULL,
    0x3fe04dca4e929c40ULL, 0x3fe05933a5558ecdULL, 0x3fe818d2ce5f63dfULL, 0x3fc2ef1bf906edbcULL, 0x3fea3a61d031713cULL,
    0x3fe5dd7c80623560ULL, 0x3fe92fe5ecd84dc6ULL, 0x3fc886e1aeefc9ccULL, 0x3fe9acf79af1c4dcULL, 0x3fc87d4d6aadd2dcULL,
    0x3fb4e0a1e07abfc8ULL, 0x3feb5e04f5a70a3cULL, 0x3feb8fa268155d33ULL, 0x3fec0c97865ae161ULL, 0x3fde33c4d2d3dfecULL,
  }}, {{0x7d526b4837e512f4ULL, 0xcc87e07aa02a8374ULL, 0xa48f128a241cf51fULL, 0x9add3b4bd45b1659ULL}}},
  Golden{2ULL, {{
    0x3fd0be40d2359ad6ULL, 0x3fd31a7a98c6fedcULL, 0x3fea0e2323ed3df6ULL, 0x3fb787cd9d738668ULL, 0x3fe3340604aab75cULL,
    0x3fe7505e2a7a05a5ULL, 0x3fc80d24727f15fcULL, 0x3fac3c2dc1c2ad30ULL, 0x3fd199191ea3b9f8ULL, 0x3fe509b0f6467179ULL,
    0x3fe1fe148f29c851ULL, 0x3fc3353d80a14ce8ULL, 0x3fdbb0390e6ff28aULL, 0x3fe56ae22b15b802ULL, 0x3fdb0ee7723c7ee2ULL,
    0x3fe4430bede70814ULL, 0x3feef53c3e1deb1eULL, 0x3fe5dbaac21bc5c5ULL, 0x3fd910619a9854baULL, 0x3fc7f7e46785178cULL,
    0x3fd6243834154fa8ULL, 0x3fe05aa707528d2eULL, 0x3fec84c9987628a0ULL, 0x3fe8d16b791c109aULL, 0x3fd45c838f3e80b8ULL,
    0x3fed932f501d09a3ULL, 0x3fde2363374e5282ULL, 0x3fe63345be52388cULL, 0x3fbb71f02bb349d0ULL, 0x3fbac35ddc7a1950ULL,
    0x3fc9d81a6dfa9a04ULL, 0x3fec4d6966ee5ef7ULL, 0x3fe5c103f750b646ULL, 0x3feb2cf1a78ffa03ULL, 0x3fe49f38cf9301a3ULL,
    0x3fda04ca679d0bf8ULL, 0x3fe087cefe32fec6ULL, 0x3fe2fd7d42991d2fULL, 0x3feb967874cf180aULL, 0x3fdc0b3dfd4da2d6ULL,
    0x3fec8d3b218f8ebfULL, 0x3fe3a391b46e176bULL, 0x3fea8a15dba272edULL, 0x3fdfe02680dc7064ULL, 0x3fe6291bc9031899ULL,
    0x3fd5b2977c2881feULL, 0x3fe0bb02d7b89d12ULL, 0x3fcbad3544874814ULL, 0x3fb9c7b615423df0ULL, 0x3fa3c3eba7008fb0ULL,
    0x3fe6765ebf90b572ULL, 0x3fdd3628c7651290ULL, 0x3fecba3cf6489087ULL, 0x3feab9d220ca4c4fULL, 0x3fd8a566101c67b2ULL,
    0x3fef2860611630baULL, 0x3fe2f22c07d9235bULL, 0x3fe8821db9636ae2ULL, 0x3fda0f78ed324778ULL, 0x3fc91c187e2e7464ULL,
  }}, {{0xc6e6c3f1be78c3f8ULL, 0xf743a71f56b6a358ULL, 0xae8316b7ae8f52d0ULL, 0xdacdda54e5a1e9c4ULL}}},
  Golden{42ULL, {{
    0x3fe8c43f79a2db24ULL, 0x3fdc16959869e47eULL, 0x3feb79a2584ddb42ULL, 0x3fe650d6c1c2c011ULL, 0x3fb81c01b5339380ULL,
    0x3fef384c5db0d8eaULL, 0x3fe85b41a5f78b48ULL, 0x3fe927705476f592ULL, 0x3fc066070b41b168ULL, 0x3fdcd31f8a75be6eULL,
    0x3fd7bb27a2e005c4ULL, 0x3feda80f0cc5d772ULL, 0x3fe49a8b06390299ULL, 0x3fea541029ace094ULL, 0x3fdc60e5f2a2e84eULL,
    0x3fcd16288f39a8a8ULL, 0x3fe1bf289862f2c7ULL, 0x3fb05653e3e34cb8ULL, 0x3fea7bf45e1b771fULL, 0x3fe436984208db37ULL,
    0x3fe842413865498eULL, 0x3fd6b08dafacd55eULL, 0x3fef0ff54da22c66ULL, 0x3fec9472beee4312ULL, 0x3fe8e88481f5b183ULL,
    0x3fc8e9ebd2614460ULL, 0x3fdddec1c5d74a3cULL, 0x3fa66d727afede80ULL, 0x3fc3bfc21145f5fcULL, 0x3fe5db897a77c9bcULL,
    0x3fe7d51771dd8559ULL, 0x3feef5d6f86bbb89ULL, 0x3fd4da529a5a4b5eULL, 0x3fd7b59ca079f44cULL, 0x3fde0d33d144e2ceULL,
    0x3fc84098f565d8d4ULL, 0x3fc0a144943b41a0ULL, 0x3fde71f3132bef42ULL, 0x3fcd0b5d94aa43dcULL, 0x3fe56f1dc2321062ULL,
    0x3fdbfa4c0abc2df8ULL, 0x3feaa54cbe85245bULL, 0x3fe668925bf42707ULL, 0x3fd3fdd0a74638f6ULL, 0x3feaa1df4e98922dULL,
    0x3fe9c0a12e8ded4bULL, 0x3fd8cc721d75de46ULL, 0x3fd273f7b8397a44ULL, 0x3fe5d700cfa7aa75ULL, 0x3fc1e368cd518168ULL,
    0x3fc996978bea971cULL, 0x3f7e27e63ce0dd80ULL, 0x3fe92e7c08397fbcULL, 0x3fe546754db8f8beULL, 0x3fe690b6fbef1713ULL,
    0x3fe8fbbb72e8de2eULL, 0x3fdd5ee045e38fb8ULL, 0x3fe23320bc902dedULL, 0x3fc1e4de3784c500ULL, 0x3fbd51d7c83ed7d8ULL,
  }}, {{0x0f194608ea37b9f2ULL, 0x7816e44699514ab2ULL, 0xa2daa89152f355b3ULL, 0x0be448f0da5f6c2bULL}}},
  Golden{100ULL, {{
    0x3feab82b65788a51ULL, 0x3fe316f87883f944ULL, 0x3fd27cbc40052308ULL, 0x3fa5fdbf8ee30e80ULL, 0x3fef282d4312cfb0ULL,
    0x3fe3164bd3be4fafULL, 0x3fe949d5f9cd2e1cULL, 0x3fed21800ddd9852ULL, 0x3fe6055c79e01c9aULL, 0x3fc851a3fe5c0c4cULL,
    0x3fef68469b702d47ULL, 0x3fd2392e54abfb2aULL, 0x3fe423017c8a5153ULL, 0x3fe297d9d044a0cdULL, 0x3fe3327b39324ee7ULL,
    0x3fe120c0a278c6acULL, 0x3fefdd67f2172d28ULL, 0x3fe00ff13c441d07ULL, 0x3fe8ac3789c40bb9ULL, 0x3fdfa058b98912f2ULL,
    0x3fefecf30f074b32ULL, 0x3fef50deeb6dca45ULL, 0x3fd9303806787d0cULL, 0x3fd49a6c6adcb7eeULL, 0x3feb96fbe079a320ULL,
    0x3fe99411cd8f9fd2ULL, 0x3fe62033225ed50dULL, 0x3fda25280f2d152aULL, 0x3fd8f1e9747dc762ULL, 0x3fc0d9dac8d26b90ULL,
    0x3fe404134eb4370aULL, 0x3fb51841972df908ULL, 0x3fd1929a95b5161aULL, 0x3fe4fee922bae4c6ULL, 0x3f8e0e7770e30840ULL,
    0x3feab47cd0440a39ULL, 0x3fb288bef87a8ae8ULL, 0x3fe0c599086759ffULL, 0x3fe17d5781ba1564ULL, 0x3fcd166db05024e8ULL,
    0x3fee975fa0da46efULL, 0x3fd5d25a0b1a5a00ULL, 0x3fe0269037fc5705ULL, 0x3febc19b46964700ULL, 0x3fe505c2314ceb3bULL,
    0x3fd89f4abbe477b2ULL, 0x3fb8a54efb47ec78ULL, 0x3fee95f5ace490f2ULL, 0x3fe2ddbbe4def340ULL, 0x3fe2a114130113a7ULL,
    0x3fe31bd066cae557ULL, 0x3fc30e4d11481bbcULL, 0x3fe1ebeb19f725c6ULL, 0x3fe20577e43f5277ULL, 0x3fe2977488b41333ULL,
    0x3fc7ac689ea934fcULL, 0x3fe6b532b86e3ceeULL, 0x3fb1ecdfaa231eb0ULL, 0x3fd4ca96507a2638ULL, 0x3fbb5ae14cb8f910ULL,
  }}, {{0x8f71518b41a75927ULL, 0x4c6a5edc1e52a147ULL, 0xad1f962a66a217d5ULL, 0x7ae3a89a87a06558ULL}}},
  Golden{4294967296ULL, {{
    0x3fec78bd7c50b582ULL, 0x3fe1d4132d1fc63bULL, 0x3fe9a109ff09b1bdULL, 0x3fee9bc2dd875173ULL, 0x3fae02ce67c855d0ULL,
    0x3fce4260c3881c34ULL, 0x3fe935c1ed2806c8ULL, 0x3f34336a40d68000ULL, 0x3fe7391f98413d73ULL, 0x3fe3ccefbdbea070ULL,
    0x3f70986207ec4900ULL, 0x3fbbb9e6f4f5f2f8ULL, 0x3fc0b460de86b714ULL, 0x3fe85fdd75c7379dULL, 0x3feeae41e6e05498ULL,
    0x3fc053d3f589e038ULL, 0x3fe28d58854ed2dbULL, 0x3fd423b0faad545aULL, 0x3fa11d96e6c27fe0ULL, 0x3fd5693cef28adf0ULL,
    0x3fd0de8fcd2fe932ULL, 0x3fe678b029c74069ULL, 0x3fdbc57a28de6568ULL, 0x3fc4cbae38ef3894ULL, 0x3fea9ebc3507adc0ULL,
    0x3fdb59fb176715aeULL, 0x3fcda03fd1522748ULL, 0x3fd051f7a5191556ULL, 0x3fd626b3c925f86eULL, 0x3fe385b602995225ULL,
    0x3fe4815f223c1de7ULL, 0x3fdecd2c0bd63fe4ULL, 0x3fe2535d630d370bULL, 0x3fed34862ff9e87cULL, 0x3fd808af7a1a25c2ULL,
    0x3f8054f28bc870c0ULL, 0x3fd8724a4e1f113aULL, 0x3fdc3e73fe65d468ULL, 0x3fb4e8622ceb12e8ULL, 0x3fede27b757e03d7ULL,
    0x3fd6b9e37353bbaeULL, 0x3f8d707f1a773280ULL, 0x3fca462d06ed3808ULL, 0x3fd2553930f97edeULL, 0x3feefd291278066fULL,
    0x3fc721d9ddcb54a8ULL, 0x3febf2c318f58b77ULL, 0x3fe2ce6fc4f6498dULL, 0x3fdb928e1c802936ULL, 0x3fe79c51b3f71b00ULL,
    0x3fef4369362ccceeULL, 0x3fca4bff19992984ULL, 0x3fe8ee61cf968047ULL, 0x3fe1c78a982a3077ULL, 0x3fbb03090a0ad288ULL,
    0x3fe16de5ecbddac6ULL, 0x3fdb7de41df01002ULL, 0x3fef81d25463afc3ULL, 0x3fd9d7ea62be486eULL, 0x3fe04a03ed11a358ULL,
  }}, {{0x9cc5f0cda1157332ULL, 0x516058142f943a52ULL, 0x1ceb3a3e046cd5ebULL, 0xe836fd1cdeedd4b7ULL}}},
  Golden{18446744073709551615ULL, {{
    0x3fe5c2c74f5188eaULL, 0x3feb0ccb3ebe6704ULL, 0x3f7e52b195bc2d00ULL, 0x3feca04d55e33445ULL, 0x3fc081eecc4d67f8ULL,
    0x3fd467d9a3936a3eULL, 0x3fe2dcf94e79f20eULL, 0x3fc7624e8c04fac8ULL, 0x3fed240c36e81e86ULL, 0x3fe05eb984a34a1fULL,
    0x3fe6278da3026e65ULL, 0x3fd11f39c30e3fd2ULL, 0x3feffb3dff9f7681ULL, 0x3fe2809170d626a8ULL, 0x3fe97369857c6cfaULL,
    0x3fd6dda38c2d2064ULL, 0x3fda33b2270cbc30ULL, 0x3fe42a06dba22bb5ULL, 0x3fe00180af081db2ULL, 0x3fe6bc0fc4336723ULL,
    0x3fe2dbb0a1e8847aULL, 0x3fb0f3dd5e93c748ULL, 0x3fe1c9b7b718231fULL, 0x3fd442e1e940011cULL, 0x3f87031e51284180ULL,
    0x3fe0472594f28614ULL, 0x3fe0d834186e7dd8ULL, 0x3fa69e2d6842ed20ULL, 0x3fd4b7794ee848a6ULL, 0x3fc11a51eff6e874ULL,
    0x3fdc381fe573e13cULL, 0x3fc09a2885a49888ULL, 0x3fe171319cf4105bULL, 0x3fe787655ef98fcaULL, 0x3fe02f6472925483ULL,
    0x3fea05a8f7873ec9ULL, 0x3fd9453317b4d846ULL, 0x3fe98d49f42127ffULL, 0x3feedc65156e7e96ULL, 0x3fbf03af1fb5c9d0ULL,
    0x3fc30f2a0a6afd5cULL, 0x3fcd97f5b66c30a4ULL, 0x3feb743e13bf1bfcULL, 0x3fee31352d3329faULL, 0x3fee25553041c0bbULL,
    0x3fbd72c891244880ULL, 0x3fe732ebccca948aULL, 0x3fc1eacc259faa68ULL, 0x3fe7d6284e912f8eULL, 0x3fd5d0473c88e074ULL,
    0x3fcf53e7a964d35cULL, 0x3fe15ad7b82a3710ULL, 0x3fe4673f72b90202ULL, 0x3fe996a32eaea088ULL, 0x3f9d94cb06a77780ULL,
    0x3fe9b5994ebd09c2ULL, 0x3fcf47bc96563da8ULL, 0x3fe52976e5d4c26aULL, 0x3fab389110ca4720ULL, 0x3fea567a5242011cULL,
  }}, {{0x5236873b9ba6ec0bULL, 0x5c2cdebba4c14a0bULL, 0x87f9cc4fd6dc3a49ULL, 0x4abbfc8a5f93335dULL}}},
}};

std::uint64_t bits(double value) {
  std::uint64_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::uint64_t hash_word(std::uint64_t hash, std::uint64_t value) {
  for (unsigned shift = 0; shift < 64; shift += 8) {
    hash ^= (value >> shift) & 255U;
    hash *= 1099511628211ULL;
  }
  return hash;
}

void test_golden() {
  for (const auto& test : golden) {
    simple_rbt::NumpyPcg64 generator(test.seed);
    for (std::size_t i = 0; i < test.unit_bits.size(); ++i) {
      const double value = generator.uniform_next();
      if (bits(value) != test.unit_bits[i]) {
        throw std::runtime_error("uniform bits mismatch: seed=" +
                                 std::to_string(test.seed) + " index=" + std::to_string(i));
      }
      if (!(value >= 0. && value < 1.)) {
        throw std::runtime_error("uniform outside [0, 1)");
      }
    }
    for (std::size_t h = 0; h < heights.size(); ++h) {
      simple_rbt::NumpyPcg64 scaled(test.seed);
      std::uint64_t hash = 14695981039346656037ULL;
      for (std::size_t i = 0; i < test.unit_bits.size(); ++i) {
        hash = hash_word(hash, bits(scaled.uniform_next() * heights[h]));
      }
      if (hash != test.scaled_hashes[h]) {
        throw std::runtime_error("scaled uniform mismatch: seed=" +
                                 std::to_string(test.seed) + " height=" + std::to_string(heights[h]));
      }
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    // Binary dump mode supports an independent Python oracle without linking
    // that oracle into the native runtime. Parse and validate the entire input.
    if (argc == 5 && std::string(argv[1]) == "--dump") {
      const std::string seed_text = argv[2];
      const std::string count_text = argv[3];
      if (seed_text.empty() || seed_text.find_first_not_of("0123456789") != std::string::npos ||
          count_text.empty() || count_text.find_first_not_of("0123456789") != std::string::npos) {
        throw std::runtime_error("seed and count must be unsigned decimal integers");
      }
      const auto seed = std::stoull(seed_text);
      const auto count = std::stoull(count_text);
      std::size_t parsed = 0;
      const std::string maximum_text = argv[4];
      const double maximum = std::stod(maximum_text, &parsed);
      if (parsed != maximum_text.size() || !std::isfinite(maximum) || maximum < 0. || count > 10000000) {
        throw std::runtime_error("invalid maximum or excessive oracle count");
      }
      simple_rbt::NumpyPcg64 generator(seed);
      for (std::uint64_t i = 0; i < count; ++i) {
        const double value = generator.uniform_next() * maximum;
        std::cout.write(reinterpret_cast<const char*>(&value), sizeof(value));
      }
      if (!std::cout) throw std::runtime_error("binary output write failed");
      return 0;
    }
    if (argc != 1) throw std::runtime_error("usage: random_oracle [--dump SEED COUNT MAX_HEIGHT]");
    test_golden();
    std::cout << "PASS NumPy 1.26.4 PCG64: 7 seeds x 60 binary64 values, "
                 "plus all 4 terrain height scales; no Python runtime dependency\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
