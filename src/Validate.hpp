#pragma once

// `hypxrcompose validate <take>`.
//
// This command is the format's arbiter: it runs the same loader the compositor
// runs, with media probing on, and prints every structural and referential
// complaint rather than stopping at the first. A bundle that validates clean is
// one `render` can consume; a bundle that only warns is one `render` will
// consume with the stated caveats.

#include <filesystem>
#include <string>

namespace hxc {

    struct SValidateOptions {
        std::filesystem::path root;
        // Promote warnings to errors, for a producer's own CI.
        bool                  strict     = false;
        bool                  jsonOutput = false;
        // Skips ffprobe. Structure only, for a quick syntax check.
        bool                  skipMedia  = false;
        // Count frames by decoding them rather than by counting the container's
        // packets. The two agree for every codec a `.hypxrtake` holds - the
        // suite asserts that they do - and the default is the fast one, because
        // a full decode of a 51-second take is a quarter of an hour. `--deep`
        // is for when a file is suspected of being truncated or corrupt.
        bool                  deep       = false;
        // md5 every decoded frame. Implies --deep.
        bool                  checksum   = false;
    };

    // 0 = clean (or warnings only, unless --strict), 1 = errors, 2 = the bundle
    // could not be opened at all.
    int runValidate(const SValidateOptions& options);

    // Human-readable one-paragraph summary of what a loaded bundle holds. Shared
    // with `render`, which prints it before composing.
    std::string describeBundle(const struct SBundle& bundle);

}
