# Tasks: Digital Twin Simulation Project: Gazebo & Unity

**Input**: Design documents from `/specs/2-digital-twin-simulation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan in simulation/ directory
- [X] T002 [P] Initialize ROS 2 workspace in simulation/ros2/ with src directory
- [X] T003 [P] Create Gazebo directory structure: simulation/gazebo/{models,worlds,plugins,launch}
- [X] T004 [P] Create Unity project structure: simulation/unity/{Assets,ProjectSettings}
- [X] T005 [P] Create documentation structure: docs/{tutorials,setup-guides}

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup Gazebo environment with ODE physics engine in simulation/gazebo/
- [X] T007 [P] Configure Unity 2021.3 LTS project with standard rendering pipeline in simulation/unity/
- [X] T008 [P] Create basic robot URDF model in simulation/ros2/src/robot_description/urdf/basic_robot.urdf
- [X] T009 Create ROS 2 packages for sensor simulation in simulation/ros2/src/sensor_simulator/
- [X] T010 Set up ROS 2 communication between environments using gazebo_ros_pkgs in simulation/ros2/src/
- [X] T011 Configure physics parameters with gravity (9.81 m/s²) in simulation/gazebo/worlds/physics_params.yaml
- [X] T012 [P] Set up documentation template using Jupyter notebook and Markdown in docs/setup-guides/

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Physics Simulation Setup in Gazebo (Priority: P1) 🎯 MVP

**Goal**: Student can set up and configure a Gazebo physics simulation environment with accurate gravity, collision detection, and robot models

**Independent Test**: Student can verify correct physics by setting up a simple object drop test and confirming gravity acceleration matches expected values (9.81 m/s²)

### Implementation for User Story 1

- [X] T013 [P] [US1] Create basic Gazebo world with accurate physics parameters in simulation/gazebo/worlds/basic_physics.world
- [X] T014 [P] [US1] Create Gazebo plugin for physics validation in simulation/gazebo/plugins/physics_validator.cc
- [X] T015 [US1] Implement collision detection test model in simulation/gazebo/models/collision_test.sdf
- [X] T016 [US1] Create launch file for basic physics simulation in simulation/gazebo/launch/physics_sim.launch.py
- [X] T017 [US1] Integrate robot model with Gazebo physics in simulation/gazebo/models/robot_with_physics.urdf
- [X] T018 [US1] Implement accurate gravity simulation with 9.81 m/s² in simulation/gazebo/worlds/gravity_params.world
- [X] T019 [US1] Create physics validation test to verify gravity tolerance in simulation/ros2/src/sensor_simulator/test/physics_validation.py
- [X] T020 [US1] Document physics setup process in docs/setup-guides/physics-setup.md
- [X] T021 [US1] Create Jupyter notebook for physics validation in docs/tutorials/physics-validation.ipynb

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Unity Visualization and Human-Robot Interaction (Priority: P2)

**Goal**: Student can create and interact with a Unity visualization of the Gazebo simulation environment with 30+ FPS performance

**Independent Test**: Student can load the Unity scene and successfully navigate around the environment, controlling the camera to view robot and scene elements from different angles

### Implementation for User Story 2

- [X] T022 [P] [US2] Create Unity scene that mirrors Gazebo environment in simulation/unity/Assets/Scenes/gazebo_mirror.unity
- [X] T023 [P] [US2] Implement camera navigation controls in simulation/unity/Assets/Scripts/CameraController.cs
- [X] T024 [US2] Import robot model into Unity scene from URDF in simulation/unity/Assets/Models/
- [X] T025 [US2] Implement interaction system for human-robot interaction in simulation/unity/Assets/Scripts/InteractionSystem.cs
- [X] T026 [US2] Set up lighting and rendering to achieve 30+ FPS in simulation/unity/Assets/Settings/
- [X] T027 [US2] Implement Unity-ROS communication using ROS# in simulation/unity/Assets/Scripts/RosCommunication.cs
- [X] T028 [US2] Create visualization of physics simulation synchronized with Gazebo in simulation/unity/Assets/Scripts/PhysicsSync.cs
- [X] T029 [US2] Document Unity visualization setup process in docs/setup-guides/unity-visualization.md
- [X] T030 [US2] Create Jupyter notebook for Unity interaction in docs/tutorials/unity-interaction.ipynb

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Sensor Simulation and Data Output (Priority: P3)

**Goal**: Student can configure and validate sensor simulations (LiDAR, Depth Camera, IMU) in Gazebo with realistic data outputs

**Independent Test**: Student can verify sensor outputs by comparing known environmental features in the simulation to sensor responses (e.g. distance measurements from LiDAR matching geometric distances)

### Implementation for User Story 3

- [X] T031 [P] [US3] Create LiDAR sensor model in simulation/gazebo/models/lidar_sensor.sdf
- [X] T032 [P] [US3] Create Depth Camera sensor model in simulation/gazebo/models/depth_camera.sdf
- [X] T033 [P] [US3] Create IMU sensor model in simulation/gazebo/models/imu_sensor.sdf
- [X] T034 [US3] Implement realistic LiDAR noise model in simulation/ros2/src/sensor_simulator/src/lidar_noise_model.cpp
- [X] T035 [US3] Implement realistic Depth Camera noise model in simulation/ros2/src/sensor_simulator/src/depth_noise_model.cpp
- [X] T036 [US3] Implement realistic IMU noise model in simulation/ros2/src/sensor_simulator/src/imu_noise_model.cpp
- [X] T037 [US3] Mount sensors on robot model in simulation/gazebo/models/robot_with_sensors.urdf
- [X] T038 [US3] Generate realistic sensor data outputs in simulation/ros2/src/sensor_simulator/src/sensor_publisher.py
- [X] T039 [US3] Validate sensor data accuracy against geometric distances in simulation/ros2/src/sensor_simulator/test/sensor_validation.py
- [X] T040 [US3] Document sensor simulation setup in docs/setup-guides/sensor-simulation.md
- [X] T041 [US3] Create Jupyter notebook for sensor validation in docs/tutorials/sensor-validation.ipynb

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Cross-Environment Synchronization

**Goal**: Ensure Unity scene mirrors Gazebo environment for consistent visualization

- [X] T042 [P] Create ROS 2 message types for state synchronization in simulation/ros2/src/sensor_simulator/msg/
- [X] T043 Implement transform synchronization between Gazebo and Unity in simulation/ros2/src/sensor_simulator/src/transform_sync.cpp
- [X] T044 Create launch file for synchronized simulation in simulation/gazebo/launch/sync_simulation.launch.py
- [X] T045 Test synchronization accuracy between environments in simulation/ros2/src/sensor_simulator/test/sync_validation.py
- [X] T046 Document synchronization setup in docs/setup-guides/synchronization.md

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T047 Create 2nd Gazebo scene to meet minimum deliverables in simulation/gazebo/worlds/advanced_physics.world
- [X] T048 Create additional Unity interactive scene to meet minimum deliverables in simulation/unity/Assets/Scenes/advanced_scenario.unity
- [X] T049 [P] Documentation updates in docs/
- [X] T050 Code cleanup and refactoring
- [X] T051 Performance optimization to ensure Unity renders at 30+ FPS
- [X] T052 [P] Asset optimization to stay under 500MB total size constraint
- [X] T053 Run quickstart.md validation in docs/setup-guides/quickstart_validation.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in priority order (P1 → P2 → P3)
- **Cross-Environment (Phase 6)**: Depends on US1 and US2 completion
- **Polish (Final Phase)**: Depends on all user stories being complete

### Within Each User Story

- Models before services
- Services before endpoints/implementation
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start after the previous one completes (P1 → P2 → P3)
- All models within a story marked [P] can run in parallel
- Different user stories must run sequentially due to shared resources but components within can be parallelized

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Cross-Environment Sync → Test → Deploy/Demo
6. Polish and optimize → Final delivery