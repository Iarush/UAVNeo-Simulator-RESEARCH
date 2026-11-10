# UAVSim, prebuilt Windows player

This branch holds the prebuilt Windows player of UAVSim (`UAVSim_Windows_v1.0.0/UAVSim.exe`). It includes its own bundled Python for the custom drone frame import, so there is nothing to install.

- Built from branch `feature/real-flight-physics` at commit `04e8217` (docs commit `013e826`).
- Windows only.
- `UAVSim_Data/sharedassets4.assets.resS` is 105 MB, over GitHub's 100 MB file limit, so it is stored with Git LFS. Install Git LFS (`git lfs install`) before you clone, or that file will be a small pointer text file and the player will not run.

To run: clone this branch and start `UAVSim_Windows_v1.0.0/UAVSim.exe`.
