# NavRep Dependency Handoff

## Installed environment snapshot
The file requirements-container.txt records the packages installed in the working Docker container. It is an environment snapshot, not a portable requirements file.

## Local and editable dependencies

The following entries need special attention when recreating the environment:

1. asl-pepper-2d-sim-maps
   Local source: /root/Code/asl_pepper_2d_sim_maps

2. pepper_2d_simulator
   Repository: https://github.com/ethz-asl/interaction_actions_for_navigation
   Commit: e38ca94127ada5a72389c661829cb2fa85db782e
   Subdirectory: external/asl_pepper/asl_pepper_2d_simulator/python

3. pyIA
   Repository: https://github.com/ethz-asl/interaction_actions_for_navigation
   Commit: e38ca94127ada5a72389c661829cb2fa85db782e
   Subdirectory: python

4. pymap2d
   Local source: /root/Code/pymap2d

5. pyniel
   Repository: https://github.com/danieldugas/pyniel
   Commit: 30988340d6894960dd838ada30a94ac19ba0d77a

6. soadrl-crowdnav
   Local source: /root/Code/dsrnn-navrep/a
   This package has a known compatibility issue with NavRep. The full training-environment import remains unresolved.

## Simulator compatibility patch

The patch is stored at:
setup/patches/pepper_2d_simulator_dtype.patch

It contains NumPy float64 compatibility changes for the Pepper simulator. Apply it only to the matching simulator source revision, after checking that the changes are not already present.

Example, from the simulator Python source directory:

    patch --dry-run -p0 < /workspace/navrep/setup/patches/pepper_2d_simulator_dtype.patch

If the dry run succeeds and the changes are not already present, apply with:

    patch -p0 < /workspace/navrep/setup/patches/pepper_2d_simulator_dtype.patch

## Important handoff warning

The paths under /root/Code are inside the currently running Docker container. They are not included automatically when the NavRep repository is shared. Preserve the container and its contents until the missing local source directories have been archived or recreated for teammates.

Do not treat requirements-container.txt as a standalone installation recipe.

## pymap2d import verification

The installed distribution is pymap2d version 0.1.15.
Its verified import modules are:

    import map2d
    import pose2d
    import CMap2D

These imports succeeded in the working container. The distribution name is pymap2d, but `import pymap2d` is not the correct module-level import check.

## Dockerfile status

The repository Dockerfile defines the base environment:
- Python 3.6.15 on Debian Buster
- Archived Debian package sources
- Build tools, Git, CMake and required native libraries
- Pinned pip, setuptools and wheel versions

The Dockerfile is not yet a complete one-command NavRep environment installation. Additional Python packages, source checkouts, native extensions, the map-resource package and the Pepper simulator compatibility patch were installed or configured separately inside the working container.

CMake has been added to the Dockerfile because it was previously installed manually in the running container.

The existing working container should be preserved until those additional installation steps are made reproducible.
