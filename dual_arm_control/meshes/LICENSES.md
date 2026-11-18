# Third-party meshes

The robot arm meshes in this folder are redistributed unmodified from the following public packages.
The URDF kinematics in `urdf/iiwa7_macro.xacro` and `urdf/sia5d_macro.xacro` are written for this repository
(from the Denavit-Hartenberg models used by the controllers). Only the visual meshes are reused.

| Folder | Source | Commit | License |
|---|---|---|---|
| `iiwa7/` | [lbr-stack/lbr_iiwa7_r800_description](https://github.com/lbr-stack/lbr_iiwa7_r800_description) `meshes/visual/` | `64a0cc38708988c631873b74070f3ee418327c68` | Apache-2.0 (as declared in the package's `package.xml`) |
| `sia5d/` | [ros-industrial/motoman](https://github.com/ros-industrial/motoman) `motoman_sia5d_support/meshes/sia5d/visual/` | `846fbbc57c60223f12bf4795ea6404982d57709e` | BSD-3-Clause (as declared in the package's `package.xml`) |

The gripper meshes (`WSG50_110.stl`, `GUIDE_WSG50_110.stl`, `WSG32.stl`, `finger.stl`) are part of this repository.

## BSD-3-Clause (motoman_sia5d_support)

Copyright (c) ROS-Industrial contributors. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Apache-2.0 (lbr_iiwa7_r800_description)

Licensed under the Apache License, Version 2.0. See the `LICENSE` file at the root of this repository for the full
text.
