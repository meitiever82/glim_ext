# Vendored: Scan Context

These sources are vendored (copied) from the Scan Context project, not tracked
as a git submodule. Only the `cpp/module/Scancontext/` files actually used by
`scan_context_loop_detector` are kept here.

- Upstream: https://github.com/koide3/scancontext.git
- Commit:   b68f8d1f077e091a4d85404265a8c69355107634
- Path:     cpp/module/Scancontext/

## Why vendored instead of a submodule

The upstream `scancontext` repository contains a dangling submodule gitlink at
`example/longterm_localization/.../PRcurve/src/npy-matlab` that has no entry in
its `.gitmodules`. This made `git clone --recursive` of glim_ext fail with
"No url found for submodule path ... npy-matlab". Since glim_ext only needs the
Scancontext core sources, those are vendored here to keep cloning reliable and
self-contained.

To update: copy `cpp/module/Scancontext/` from the upstream commit you want and
update the commit hash above.
