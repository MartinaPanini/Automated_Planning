FROM ros:humble

# --- CONFIGURAZIONE BASE & ROS ---
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/London"

# Installazione dipendenze base
RUN apt-get update && apt-get install -y \
    software-properties-common \
    python3-pip \
    python3-venv \
    curl \
    wget \
    unzip \
    vim \
    gdb \
    terminator \
    git \
    && rm -rf /var/lib/apt/lists/*

# --- INSTALLAZIONE PLANUTILS & APPTAINER ---
RUN add-apt-repository -y ppa:apptainer/ppa && \
    apt-get update && apt-get install -y apptainer

RUN pip3 install planutils

# Setup planutils
RUN planutils setup
RUN planutils install -y val
RUN planutils install -y ff
RUN planutils install -y metric-ff
RUN planutils install -y enhsp
RUN planutils install -y popf
RUN planutils install -y optic
RUN planutils install -y tfd
RUN planutils install -y downward

# Config hostfs
RUN if [ -f /etc/apptainer/apptainer.conf ]; then \
    perl -pi.bak -e "s/mount hostfs = no/mount hostfs = yes/g" /etc/apptainer/apptainer.conf; \
    fi

# --- INSTALLAZIONE PLANSYS2 (ROS2) ---
# Installiamo i pacchetti ROS base
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# Setup Workspace Plansys2
RUN mkdir -p ~/plansys2_ws/src && \
    cd ~/plansys2_ws/src && \
    git clone https://github.com/PlanSys2/ros2_planning_system.git && \
    cd ros2_planning_system && \
    git checkout ${ROS_DISTRO}-devel

# CORREZIONE QUI: Inizializziamo rosdep e aggiorniamo apt PRIMA di installare
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update && \
    apt-get update && \
    cd ~/plansys2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

RUN cd ~/plansys2_ws/src && \
    git clone https://github.com/PlanSys2/ros2_planning_system_examples.git && \
    cd ros2_planning_system_examples && \
    git checkout ${ROS_DISTRO}

# CORREZIONE QUI: Aggiorniamo apt anche per il secondo rosdep
RUN apt-get update && \
    cd ~/plansys2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

# Build colcon finale
RUN cd ~/plansys2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

# UX
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ~/plansys2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]