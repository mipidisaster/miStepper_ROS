# miStepper_ROS
This library contains the support for controlling the miStepper via ROS (the intended way to interface with this device).

# Linked libraries:
## milibrary
This library makes use of an extra repository - ['milibrary'](https://github.com/mipidisaster/miLibrary), which is not included within this repository; due to it being a seperate one.. So this will need to be included...
> It doesn't matter were this is stored, however it should be at a high enough level that any lower level scripts can make reference to it.
> To avoid any ROS build issues, suggest that this be placed within the initial 'src' of workspace
```
<ROS works>
  ├─ < other folders >
  └─ src
      ├─ milibrary
      │      └─ <contents>
      └─ <other packages>
```
Additionally, the file(s) `milibrary-CMakeLists.txt` and `milibrary-package.xml` will need to be placed within the `milibrary` folder, and renamed to `CMakeLists.txt`/`package.xml`.

## miStepper
As this library is creating the ROS infrastructure for the `miStepper` it is natural that this will be interfacing with this library. No manual changes are needed for this library

# Stack overview
## open_loop
> To be added


# Build Tree
![build tree](/_image/Build_tree.png)

v0.1.1
> In work...

v0.1.0
* Initial release (to be used with [milibrary v0.1.0](https://github.com/mipidisaster/miLibrary/tree/v0.1.0) and [miStepper v0.1.0](https://github.com/mipidisaster/miStepper))

# Wiki
The wiki for this can be found within the Github repository for this - > to be added

See https://github.com/mipidisaster/miLibrary/wiki for basic information on Eclipse setup/remote ROS downloading...

# Issues
If you find any issues with this library, please capture them within the Github repository issue area - https://github.com/mipidisaster/miStepper_ROS/issues.