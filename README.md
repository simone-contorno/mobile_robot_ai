# Mobile Robot AI

A Python-based ROS2 simulation using Nav2 to operate a unicycle robot in a simulated environment. This repository enables path planning and control input generation using multiple methods, including PID, OpenAI's API, and machine learning models such as linear regression. The goal is to compare these methods in terms of control input accuracy and performance.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Control methods](#controls)
- [How it works](#how)
- [Prerequisites](#pre)
- [Installation](#install)
- [Configuration](#config)
- [Execution](#exec)
- [Known Problems](#known)
- [Further Improvements](#improve)

<a name="overview"></a>

## Overview

This project leverages ROS2's Nav2 stack to generate paths in a simulated environment, which are then followed by a unicycle robot. The control inputs required to follow the path are produced through various techniques, allowing experimentation and comparative analysis among PID, machine learning models, and OpenAI's API.

<a name="features"></a>

## Features
<ul>
<li><b>ROS2 & Nav2 Integration</b>: Generates paths for a unicycle robot in a simulated environment.</li>
<li><b>Multiple Control Methods</b>:
    <ul>
        <li><b>PID Control</b>: Generates control inputs through a proportional-integral-derivative approach.</li>
        <li><b>OpenAI API Integration</b>: Uses OpenAI API with PID system data for control generation.</li>
        <li><b>Machine Learning</b>: Predicts control inputs using linear regression models trained on PID controller data.</li>
    </ul>
</li>
<li><b>Data-Driven Approach</b>: Leverages PID data (position and control inputs) to train machine learning models for improved path-following accuracy.</li>
<li><b>Comparative Analysis</b>: Enables performance evaluation among various control methodologies.</li>
</ul>

<a name="controls"></a>

## Control methods

<li><b>PID Control</b>: Generates control inputs based on proportional, integral, and derivative terms. PID control provides a benchmark for evaluating other methods.</li>
<li><b>OpenAI API</b>: Uses API calls with PID data as inputs, generating controls based on external inference.</li>
<li><b>Linear Regression</b>: A machine learning approach trained on PID data to predict control inputs with a regression model.</li>

<a name="how"></a>

## How it works

<li>The program subscribes to goal pose, laser scanner, and path topics.</li>
<li>The path is then modified by altering a user-defined percentage of points and republished on a different topic. This modification reduces the path's complexity.</li>
<li>Then, the control inputs are computed using a PID controller approach.</li>

The PID orientation is computed to orient the robot always againts the goal point.

<a name="pre"></a>

## Prerequisites
To run the simulation, the following packages must be installed:
<ul>
    <li><a href="https://docs.nav2.org/getting_started/index.html">navigation2</a></li>
    <pre><code>sudo apt install ros-[ros2-distro]-navigation2</pre></code>
    <li><a href="https://docs.nav2.org/getting_started/index.html">nav2-bringup</a></li>
    <pre><code>sudo apt install ros-[ros2-distro]-nav2-bringup</pre></code>
    <li><a href="https://docs.nav2.org/getting_started/index.html">turtlebot3-gazebo</a></li>
    Jazzy and newer:
    <pre><code>sudo apt install ros-[ros2-distro]-nav2-minimal-tb*</pre></code>
    Iron and older:
    <pre><code>sudo apt install ros-[ros2-distro]-turtlebot3-gazebo</pre></code>
    <li><a href="https://index.ros.org/p/tf_transformations/">tf-transformations</a></li>
    <pre><code>sudo apt install ros-[ros2-distro]-tf-transformations</pre></code>
</ul>

To use the ML methods, you need to install the following packages:
<ul>
    <li><a href="https://pandas.pydata.org/docs/getting_started/install.html">Pandas</a></li>
    <pre><code>pip install pandas</pre></code>
    <li><a href="https://pypi.org/project/transforms3d/0.4.1/">transforms3d 0.4.1</a></li>
    <pre><code>pip install transforms3d==0.4.1</pre></code>
    <li><a href="https://numpy.org/install/">NumPy 1.26.4</a></li>
    <pre><code>pip install numpy==1.26.4</pre></code>
    <li><a href="https://scikit-learn.org/stable/install.html">scikit-learn</a></li>
    <pre><code>pip install scikit-learn</pre></code>
    <li><a href="https://pypi.org/project/openai/">OpenAI</a></li>
    <pre><code>pip install openai</pre></code>
</ul>

You can easily install all of these packages by running the <i><b>config.sh</b></i> file, passing the ROS2 version name as argument.

In order to use the OpenAI API you need to <a href="https://platform.openai.com/docs/quickstart">create and set the API key</a>.<br>

<a name="install"></a>

## Installation 

<ol>
    <li>Go into the src folder of your ROS 2 workspace.<br></li> 
    <li>Download the repository:
    <pre><code>git clone https://github.com/simone-contorno/mobile_robot_ai</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and build it: 
    <pre><code>colcon build --packages-select mobile_robot_ai</code></pre>
    </li>
</ol>

<a name="config"></a>
# Configuration
The program can be configured using the <i><b>config.json</b></i> file by:
<ul>
    <li>Tuning the PID.</li>
    <li>Change the goal threashold and weights.</li>
    <li>Choosing the control mode:
        <ul>
            <li><b>0</b> for manual PID control.</li>
            <li><b>1</b> for OpenAI API-generated PID control (set the AI model and the system prompt).</li>
            <li><b>2</b> for Linear Regression control (set the dataset).</li>
        </ul>
    </li>
</ul>
The system prompts are specified in the <i><b>mobile_robot_ai/ai_prompts.py</b></i> file.<br>

<a name="exec"></a>

## Execution

To run the environment simulation you first need to configure it:
<pre><code>export TURTLEBOT3_MODEL=waffle ;
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix mobile_robot_ai)/share/mobile_robot_ai/worlds/ 
</code></pre>

Then, you can launch it:
<pre><code>ros2 launch mobile_robot_ai simulation_launch.py cmd_vel_remap:=/mobile_robot_ai/cmd_vel plan_remap:=/mobile_robot_ai/plan
</code></pre>
Here the remap is necessary to consider the modified plan by <bi>data_subscriber.cpp</bi> node.

Finally, you can run the following nodes:
<pre><code>ros2 run mobile_robot_ai data_subscriber ; 
ros2 run mobile_robot_ai control_handler.py ;
ros2 run mobile_robot_ai [control_method].py 
</code></pre>

To easily execute all this commands, run the <i><b>run.py</b></i> file.

<a name="known"></a>

## Known Problems

`cmd_vel` remapping doesn't work from ROS 2 Humble onward.

From ROS 2 Humble, remappings set in a parent launch file (e.g. with `SetRemap`) **do not propagate** into included launch files like `bringup_launch.py`, unless the included file explicitly supports them via `LaunchArgument` and passes them to `Node(..., remappings=[...])`.

As a result, remapping `cmd_vel` externally has **no effect** on Nav2's `controller_server` unless you use a patched version of the launch file.

<a name="improve"></a>

## Further Improvements

Possible future improvements include:
<ul>
    <li>Enhancing the logic of the prompts and rewriting them in XML format.</li>
    <li>If the necessary resources are available, attempting to generate control inputs using a locally installed Large Language Model (LLM).</li> 
    <li>Extend machine learning approaches to more complex models beyond linear regression.</li>
    <li>Perform a comparative analysis between the control methods, assessing accuracy, computational efficiency, and robustness.</li>
</ul>
