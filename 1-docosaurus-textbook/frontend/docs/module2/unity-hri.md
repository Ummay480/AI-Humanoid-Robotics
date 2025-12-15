# Unity for Human-Robot Interaction (HRI)

When robots interact with people, **perception matters**. **Unity**, a professional game engine, excels at photorealistic rendering, social robotics, and VR/AR telepresence. In this lesson, you'll learn how to use Unity for HRI simulations and integrate it with ROS 2.

---

## Why Unity for HRI?

### Photorealistic Visuals

**Problem**: Gazebo's graphics aren't realistic enough for vision-based AI.
- Simplified lighting (no dynamic shadows, reflections)
- Low-poly models
- Limited material properties (no subsurface scattering, PBR)

**Unity Solution**:
- **HDRP/URP**: High-definition rendering (ray tracing, global illumination)
- **Asset Store**: Thousands of realistic environments (homes, offices, warehouses)
- **Character Models**: Realistic humans for social robotics testing

### Reinforcement Learning

**Unity ML-Agents** is a toolkit for training RL agents:
- **PPO, SAC** algorithms built-in
- **Multi-agent training** (cooperation, competition)
- **Imitation learning** (learn from demonstrations)
- **Curriculum learning** (progressively harder tasks)

### VR/AR Support

**Unity excels at immersive experiences**:
- **VR Telepresence**: Control robot from Oculus/HTC Vive
- **AR Visualization**: Overlay robot plans on real world (HoloLens)
- **User Studies**: Test HRI scenarios with human participants

---

## Unity vs. Gazebo: Quick Comparison

| Feature | Unity | Gazebo |
|---------|-------|--------|
| **Graphics** | Photorealistic | Functional |
| **Physics** | Good (PhysX, Havok) | Excellent (ODE, Bullet, DART) |
| **ROS 2 Integration** | TCP/UDP bridge | Native |
| **RL Training** | ML-Agents (excellent) | Manual setup |
| **VR/AR** | Native | Limited |
| **Learning Curve** | Moderate (GUI-based) | Steep (config files) |
| **Cost** | Free (personal), paid (pro) | Free (open-source) |

**Bottom Line**: Use Unity for perception-heavy tasks and HRI; use Gazebo for physics-critical robotics.

---

## Setting Up Unity for Robotics

### Installation

1. **Download Unity Hub**: [unity.com/download](https://unity.com/download)
2. **Install Unity Editor**: Version 2021.3 LTS (or newer)
3. **Install Unity Robotics Hub**:
   ```bash
   # In Unity: Window → Package Manager → + → Add package from git URL
   # Add: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

### Unity Robotics Hub Components

1. **ROS-TCP-Connector**: Sends/receives ROS 2 messages over TCP
2. **URDF Importer**: Loads URDF models into Unity
3. **Articulation Body**: Unity's physics for robots (joints, motors)

---

## Connecting Unity to ROS 2

**Architecture**:
```
┌──────────────┐         TCP/UDP          ┌──────────────┐
│   Unity      │ <──────────────────────> │   ROS 2      │
│  (Sim/VR)    │    ROS-TCP-Endpoint      │  (Control)   │
└──────────────┘                          └──────────────┘
```

### Step 1: Install ROS-TCP-Endpoint

```bash
# In your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Launch ROS-TCP-Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

**Expected Output**:
```
[INFO] [ros_tcp_endpoint]: ROS-TCP endpoint starting on 127.0.0.1:10000
```

### Step 3: Configure Unity

In Unity:
1. **Robotics → ROS Settings**
2. Set ROS IP Address: `127.0.0.1`
3. Set ROS Port: `10000`
4. Protocol: `TCP/IP`

### Step 4: Test Connection

**Unity Script** (C#):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity/test";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            StringMsg msg = new StringMsg("Hello from Unity!");
            ros.Publish(topicName, msg);
            Debug.Log("Published message to ROS");
        }
    }
}
```

**Test**:
1. Attach script to a GameObject in Unity
2. Press Play
3. Press Spacebar
4. In ROS 2 terminal: `ros2 topic echo /unity/test`

**Expected Output**:
```
data: 'Hello from Unity!'
```

---

## Importing URDF into Unity

### Step 1: Prepare URDF

```bash
# Place your URDF and meshes in Unity Assets folder
# Example: Assets/Robots/MyRobot/my_robot.urdf
```

### Step 2: Import

In Unity:
1. **Assets → Import Robot from URDF**
2. Select your `.urdf` file
3. Configure import settings:
   - **Axis Type**: Y Axis (Unity standard)
   - **Mesh Decomposer**: VHACD (for complex collisions)
4. Click **Import**

**Result**: Robot appears in Scene with ArticulationBody components!

---

## Unity ML-Agents for Humanoid Training

### Example: Training a Humanoid to Walk

**Scenario**: Train a simulated humanoid to walk forward using reinforcement learning.

### Step 1: Install ML-Agents

```bash
pip install mlagents
```

### Step 2: Create Agent Script

```csharp
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class WalkingAgent : Agent
{
    public Transform target;  // Goal position
    Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset robot position
        transform.localPosition = new Vector3(0, 1, 0);
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Randomize target position
        target.localPosition = new Vector3(Random.Range(-5f, 5f), 0.5f, Random.Range(5f, 10f));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observe robot state
        sensor.AddObservation(transform.localPosition);  // 3 values
        sensor.AddObservation(rb.velocity);  // 3 values
        sensor.AddObservation(target.localPosition);  // 3 values
        // Total: 9 observations
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Apply forces based on agent's actions
        float forceX = actions.ContinuousActions[0];  // -1 to 1
        float forceZ = actions.ContinuousActions[1];  // -1 to 1

        rb.AddForce(new Vector3(forceX, 0, forceZ) * 10f);

        // Reward for moving toward target
        float distanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);
        if (distanceToTarget < 1.5f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        // Penalty for falling
        if (transform.localPosition.y < 0.5f)
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
```

### Step 3: Train the Agent

```bash
# In Unity: Configure ML-Agents behavior (create .yaml config)
# Then run training:
mlagents-learn config/humanoid_walk.yaml --run-id=HumanoidWalk001

# Press Play in Unity when prompted
```

**Training Output**:
```
Step: 50000. Mean Reward: 0.45. Std: 0.12.
Step: 100000. Mean Reward: 0.78. Std: 0.08.
...
Training complete! Model saved to results/HumanoidWalk001/
```

---

## VR Telepresence in Unity

### Example: Control Robot from VR Headset

**Use Case**: Operator wears VR headset and controls a remote humanoid robot.

**Setup**:
1. **Unity XR Plugin**: Install Oculus/OpenXR
2. **VR Rig**: Add XR Origin (camera + controllers)
3. **ROS Bridge**: Send controller poses to ROS 2

**Code Snippet** (Send VR Hand Pose to ROS):
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VRHandPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Transform rightHand;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>("/vr/right_hand/pose");
    }

    void FixedUpdate()
    {
        PoseMsg pose = new PoseMsg();
        pose.position = new PointMsg
        {
            x = rightHand.position.x,
            y = rightHand.position.y,
            z = rightHand.position.z
        };
        pose.orientation = new QuaternionMsg
        {
            x = rightHand.rotation.x,
            y = rightHand.rotation.y,
            z = rightHand.rotation.z,
            w = rightHand.rotation.w
        };

        ros.Publish("/vr/right_hand/pose", pose);
    }
}
```

**ROS 2 Side** (receive and map to robot):
```python
# Subscribe to VR hand pose and map to robot arm IK
```

---

## Mini Exercise: Build a Social Robot Scene

**Challenge**: Create a Unity scene where a humanoid robot interacts with a virtual human.

**Requirements**:
1. Import a humanoid robot URDF
2. Add a human character (from Unity Asset Store or free model)
3. Create a simple environment (room with furniture)
4. Implement a "wave detection" script:
   - Detect when human's hand is raised (Y > 1.5m)
   - Robot responds by waving back (animate arm joints)
5. Publish interaction event to ROS 2 topic `/social/wave_detected`

**Hints**:
- Use Unity's Animation system for human gestures
- Use ArticulationBody.xDrive to move robot joints
- Check human's hand position each frame

**Extension**: Add speech bubbles with Unity UI!

---

## Summary

You've explored Unity for human-robot interaction:

✅ **Unity excels** at photorealistic visuals, social robotics, VR/AR
✅ **ROS-TCP-Connector** bridges Unity and ROS 2 via TCP/UDP
✅ **URDF Importer** loads robot models into Unity
✅ **ML-Agents** enables RL training (walking, manipulation)
✅ **VR telepresence** allows remote robot control
✅ **Social robotics** benefits from realistic human models and environments

**Next Up**: Deep dive into **LIDAR sensors** for navigation and mapping!

Ready to scan the world? Let's continue! 📡🤖
