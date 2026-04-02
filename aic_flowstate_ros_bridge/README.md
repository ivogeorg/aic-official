# icon_bridge

A Flowstate-ROS bridge plugin that bridges the AgentBridge and AgentBridgeJoint actions on ICON to ROS.

## Quickstart (Sideload as Flowstate Service)

### Build the icon_bridge bundle (Docker)

Set up the Docker engine:

```bash
cd ~/ws_aic
./src/sdk-ros/scripts/setup_docker.sh
```

Then, create the service bundle using the `build_service_bundle.sh` script.

```bash
cd ~/ws_aic
./src/sdk-ros/flowstate_ros_bridge/scripts/build_service_bundle.sh --ros_distro kilted
```

### Sideload the flowstate_ros_bridge service into Flowstate

```bash
# 1. Export path to side-loaded service bundle
export SERVICE_BUNDLE=~/ws_aic/images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SERVICE_BUNDLE
```
