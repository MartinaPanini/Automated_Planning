# Automated Planning - Interplanetary Museum Vault (IMV)

**Student:** Martina Panini  
**Course:** Automated Planning: Theory and Practice (2025/2026)

Repository for the **Interplanetary Museum Vault (IMV)** project. This repository contains the solution for the Automated Planning assignment, covering Classical, Optimal, Hierarchical (HTN), and Temporal planning, culminating in a ROS 2 integration using **PlanSys2**.

## 📂 Repository Structure

* **`Dockerfile`**: Environment definition (ROS 2 Humble + Planutils + PlanSys2).
* **`Planners/`**: PDDL/HDDL domains and problems for scenarios 1 through 4.
    * `Problem 1`: Classical Planning (LAMA).
    * `Problem 2`: Optimal Planning (Fast Downward).
    * `Problem 3`: HTN Planning (Panda).
    * `Problem 4`: Temporal Planning (Optic).
* **`plansys2_ws/`**: ROS 2 Workspace for Problem 5.
    * `src/imv_problem_5`: Custom package containing the PDDL 2.1 domain, Lifecycle Nodes (C++), and Launch files.

---

## 🛠️ Installation & Setup

The project is fully containerized to ensure reproducibility.

### Prerequisites
* **Docker** installed on your machine.
* (Optional) An X11 server if you intend to use GUI tools (e.g., `rqt`), though the project runs fully in the terminal.

### 1. Build the Docker Image
From the root of this repository, build the image. This installs ROS 2, compiles PlanSys2, and sets up Planutils.

```bash
docker build -t imv_planning .
```

### 2. Run the Docker Container

```bash
docker run -it --rm --name imv_container \
    -v $(pwd):/root/Automated_Planning \
    imv_planning
```

## ✅ Running the planners
### Problem 1 - Classical Planning (LAMA)
```bash
cd /root/Automated_Planning/Planners/Problem\ 1
downward --alias lama-first domain_1.pddl problem_1.pddl
```

```bash
planutils run lama-first domain_1.pddl problem_1.pddl
```

### Problem 2 - Optimal Planning (Fast Downward)
```bash
cd /root/Automated_Planning/Planners/Problem\ 2
downward --alias seq-opt-lmcut domain_2.pddl problem_2.pddl
```

```bash
planutils run downward -- --alias seq-opt-lmcut domain_2.pddl problem_2.pddl
```

### Problem 3 - HTN Planning (Panda)
```bash
cd /root/Automated_Planning/Planners/Problem\ 3
downward --alias htn domain_3.pddl problem_3.pddl
```

```bash
planutils run htn domain_3.pddl problem_3.pddl
```

### Problem 4 - Temporal Planning (Optic)
```bash
cd /root/Automated_Planning/Planners/Problem\ 4
downward --alias optic domain_4.pddl problem_4.pddl
```

```bash
planutils run optic domain_4.pddl problem_4.pddl
```

## Running PlanSys2
```bash
cd /root/Automated_Planning/plansys2_ws
colcon build --symlink-install --packages-select imv_problem_5
source install/setup.bash
ros2 launch imv_problem_5 imv_problem_5.launch.py
```

Inside terminal 2:

```bash
docker exec -it imv_container bash
```

```bash
source ~/plansys2_ws/install/setup.bash
ros2 run plansys2_terminal plansys2_terminal
```

```bash
ros2 run plansys2_terminal plansys2_terminal < /root/Automated_Planning/plansys2_ws/src/imv_problem_5/pddl/setup_problem.txt
```

