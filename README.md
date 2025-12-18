# 🚁 MARL-SmartFleet-SAR: Autonomous Search & Rescue Swarm

> **A Multi-Agent Reinforcement Learning (MARL) system for decentralized Search and Rescue operations, featuring a Hybrid Executive Architecture (RL + Search).**

## 📖 Project Overview

This project simulates a fleet of autonomous drones tasked with rescuing victims in a hazardous, debris-filled environment. The system addresses the "non-stationarity" problem in multi-agent learning by evolving from independent Q-Learning to a robust **Hysteretic Q-Learning** model.

To achieve real-time performance, the system utilizes a **Hybrid Executive Architecture**:

* **The Brain (Decision Layer):** Uses Hysteretic Q-Learning with Manhattan Heuristics ($O(1)$) for instant task bidding.
* **The Legs (Execution Layer):** Uses Breadth-First Search (BFS) for precise, collision-free navigation.

## 🚀 Key Features

* **Hysteretic Q-Learning:** Dual learning rates ($\alpha > \beta$) to stabilize cooperation and ignore teammate errors.
* **Hybrid Navigation:** Decoupled decision-making (Heuristic) and movement (BFS) for smooth, lag-free performance.
* **Battery Awareness:** Agents dynamically accept/reject tasks based on energy reserves and "return-to-home" viability.
* **Self-Healing Swarm:** Healthy agents automatically detect and rescue "dead" drones to restore fleet capacity.
* **XAI Dashboard:** A custom GUI with "Cyber-Tactical" logs displaying real-time Q-Values and decision logic.

## 🛠️ Installation & Usage

### Prerequisites
* MATLAB (R2020b or later recommended).
* No additional toolboxes required (Standard Base MATLAB).

### How to Run

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/YOUR_USERNAME/MARL-SmartFleet-SAR.git
    ```

2.  **Open MATLAB** and navigate to the project folder.

3.  **Run the main script** for the final optimized version:
    ```matlab
    >> Heurestic Q Learning with BFS and Mahattan Distance_Final model (Phase6).m
    ```

4.  **Control Panel Instructions:**
    * Click **"Initialize Neural Policy"** to train the agents (populates Q-Tables).
    * Select a map (e.g., *"Tsunami Aftermath"*).
    * Click on the map to place victims.
    * Click **"Initiate Deployment"** to watch the swarm operate.
