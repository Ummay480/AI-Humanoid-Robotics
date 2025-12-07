# Service Contracts: Chapter 1 — The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Status**: Complete
**Purpose**: Define all ROS 2 Service contracts (request/response interfaces) for chapter examples

---

## Service Contract Pattern

Each service specifies:
- **Name**: ROS service name (e.g., `/add_two_ints`)
- **Service Type**: Full type (e.g., `example_interfaces/srv/AddTwoInts`)
- **Server**: Which node implements the service
- **Client(s)**: Which node(s) call the service
- **Timeout**: Maximum wait time for response
- **Validation**: Request/response validation rules
- **User Story**: Which story exercises this contract

---

## Service Contract 1: AddTwoInts (Math Service)

```yaml
Name: /add_two_ints
Service Type: example_interfaces/srv/AddTwoInts
Purpose: Demonstrate synchronous request/response communication (User Story 3)

Server:
  - service_server.py (User Story 3, example: 04_service_server.py)
  - Listens on /add_two_ints
  - Processes requests and responds with sum

Client(s):
  - service_client.py (User Story 3, example: 05_service_client.py)
  - Sends requests; waits for response
  - Demonstrates blocking call pattern

Service Definition (from example_interfaces):
  Request:
    a: int64  # First operand
    b: int64  # Second operand
  Response:
    sum: int64  # Sum of a + b

Request Validation:
  - Both a and b must be valid int64 values
  - No range constraints (can be negative, zero, or positive)
  - Valid examples: {a: 5, b: 3}, {a: -10, b: 20}, {a: 0, b: 0}
  - Invalid examples: a or b is NaN or infinity (int64 cannot represent these)

Response Validation:
  - sum must be valid int64
  - Calculated as: sum = a + b
  - No overflow handling (assume values fit in int64)

Timeout & Reliability:
  - Timeout: 5 seconds (ROS 2 default)
  - If server doesn't respond within timeout, client gets exception
  - Server should respond immediately (computation is trivial)

Error Handling:
  - If server is not running: Client will raise exception "Service not available"
  - If request is malformed: ROS 2 serialization will reject
  - If response is malformed: ROS 2 deserialization will reject

Usage Example:
  ```python
  # Server Side (User Story 3, example: 04_service_server.py)
  def add_callback(request, response):
      response.sum = request.a + request.b
      return response

  srv = node.create_service(
      AddTwoInts,
      '/add_two_ints',
      add_callback
  )

  # Client Side (User Story 3, example: 05_service_client.py)
  client = node.create_client(AddTwoInts, '/add_two_ints')
  while not client.wait_for_service(timeout_sec=1.0):
      print('Service not available, waiting...')

  request = AddTwoInts.Request()
  request.a = 5
  request.b = 3

  future = client.call_async(request)
  rclpy.spin_until_future_complete(node, future)

  response = future.result()
  print(f"5 + 3 = {response.sum}")  # Output: 5 + 3 = 8
  ```

Mapping to User Stories:
  - US1: Mentioned in architecture section as example of Services
  - US3: Core example demonstrating synchronous communication
  - US4: Optional; agent could use service for policy decisions (not in baseline)
  - US6: Not used in pipeline (publish/subscribe preferred for continuous control)

Key Learning Points:
  - Services are synchronous (client blocks until response)
  - Suitable for request/response patterns (e.g., "compute something once")
  - Not suitable for continuous control (Topics are better)
  - Error handling important (service might not be available)

```

---

## Service Interaction Diagram

```
User Story 3: Services

Server Process:
┌──────────────────────┐
│ service_server.py    │
│ ─────────────────── │
│ def add_callback():  │
│   sum = a + b        │
│   return response    │
└──────────────────────┘
         │ listens on
         v
    /add_two_ints service


Client Process:
┌──────────────────────┐
│ service_client.py    │
│ ─────────────────── │
│ 1. Creates client    │
│ 2. Waits for service │
│ 3. Creates request   │
│ 4. Calls service     │
│ 5. Waits for response│
│ 6. Prints result     │
└──────────────────────┘
         │ calls
         v
    /add_two_ints service
         │ returns
         v
    Response: {sum: 8}
```

---

## Comparison: Topics vs. Services (User Story 3)

### When to Use Topics (Asynchronous):
- Continuous data flow (sensors, telemetry)
- Multiple subscribers possible
- Decoupled publisher/subscriber (don't need to know about each other)
- Can tolerate slight delays or message loss (best-effort QoS)
- Example: `/sensor_data`, `/robot_commands`

### When to Use Services (Synchronous):
- One-time requests requiring immediate response
- Request/response pairs only
- Tight coupling: client and server both needed
- Must be reliable (Reliable QoS implied)
- Example: `/add_two_ints`, configuration services

### Example Decision Tree:
```
Is this continuous data?
  ├─ YES → Use Topic
  └─ NO: Is this a request for something?
    ├─ YES, need immediate response → Use Service
    ├─ YES, don't need response → Use Topic (publish-only)
    └─ NO → Reconsider your design
```

---

## Service Extensions (Optional for Advanced Readers)

### Potential Service 2: GetJointLimits (Optional)

```yaml
Name: /get_joint_limits
Service Type: [Custom - not in baseline]
Purpose: Query joint limits from URDF (optional extension for User Story 6)

Request:
  joint_name: string  # Name of joint (e.g., "torso_left_hip")

Response:
  lower_limit: float64  # Minimum angle in radians
  upper_limit: float64  # Maximum angle in radians
  velocity_limit: float64  # Max angular velocity rad/s
  effort_limit: float64  # Max torque N⋅m

Usage Context:
  - Controller might query this at startup to understand robot constraints
  - Not required for User Story 6 MVP
  - Mentioned only as extension example

Note: This service requires custom message definition (srv file).
Not included in core chapter to minimize complexity.
```

---

## Summary

Service contracts in this chapter:

✅ **Minimal scope** — Only one required service (AddTwoInts) for User Story 3 demonstration
✅ **Standard types** — Uses ROS 2 example_interfaces (no custom definitions needed)
✅ **Clear semantics** — Request/response validation and error handling documented
✅ **Educational focus** — Demonstrates synchronous communication pattern without overwhelming complexity
✅ **Extensible** — Optional advanced services suggested but not required

Services in this chapter focus on teaching the synchronous request/response pattern. Continuous control (User Story 4–6) uses Topics instead, which is the preferred pattern for robotics.
