# High Rollers

Samson Tsegai, Marissa Shaffer, and Justin Xu

Final Project Repo for Spring 2026, CSCI2240: Advanced Computer Graphics at Brown University.

## Project Description

We implemented a version of the method described in the 2025 SIGGRAPH Paper, "Putting Rigid Bodies to Rest" by Keenan Crane, Qingnan Zhou, Nicholas Sharp, Hossein Baktash, and Alec Jacobson. https://hbaktash.github.io/projects/putting-rigid-bodies-to-rest/

## Project Presentation

[High Rollers.pptx](https://github.com/user-attachments/files/27497098/High.Rollers.pptx)

## Results

**Cow Mesh Rolling to Rest**
<img width="676" height="217" alt="cow_roll" src="https://github.com/user-attachments/assets/8e65e466-6ede-4fc5-bdaf-7d4c2e63b4e1" />

**Cube Rolling to Rest**
<img width="301" height="211" alt="Screenshot 2026-05-07 at 5 30 01 PM" src="https://github.com/user-attachments/assets/2f9e4426-d704-4834-a328-1f0b4f1aaea3" />

**Gauss Map Visualization**
<img width="404" height="215" alt="Screenshot 2026-05-07 at 5 29 31 PM" src="https://github.com/user-attachments/assets/ea85492b-a557-4c80-9467-e7783a9806e0" />



## Setup

- After cloning the repository to your local device, make sure to run this command in the project directory in your terminal:
  - `git submodule update --init --recursive --depth 1`
    - _Tip:_ The `--depth 1` flag will reduce the memory footprint of the dependencies on your device.
- _Note for Windows users:_ To build the project, you will need to run your IDE or build software of your choice (e.g. CMake) in administrator mode.
  - This is because symlinks are used during the build process, a privileged operation.
- If you are using QtCreator, you can set your default working directory to be `%{Project:Path}` in the Build & Run settings in Preferences to make your life easier.

## Usage

- The executable requires one parameter: a path to a mesh file. It can be of type:
  - `.obj`
  - `.ply`
  - `.stl`
  - `.off`
- After passing a mesh file path, you can pass three optional floating-point numbers that represent the xyz-coordinates of the center of mass of the mesh.
- You can call the executable in the terminal with the `--help` or `-h` flags for a breakdown of the arguments.

## Notes

- Here is a link to the [shorthand reference notes](ref-notes.md) for the project.

## Dependencies

This project acknowledges and expresses thanks for the use of the following external libraries:

- [**Geometry Central** by Nicholas Sharp and Keenan Crane and others](https://geometry-central.net)
- [**Polyscope** by Nicholas Sharp and others](www.polyscope.run)
- [**Eigen** by Gaël Guennebaud and Benoît Jacob and others](https://libeigen.gitlab.io)
- [**Quickhull** by Antti Kuukka and others](https://github.com/akuukka/quickhull)
- **[Bullet](https://github.com/bulletphysics/bullet3)**
