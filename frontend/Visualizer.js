export class Visualizer {
    constructor(rosConnection, divID) {
      this.ros = rosConnection;
      this.divID = divID;
    }
  
   init() {
      // Create the main viewer.
      let viewer = new ROS3D.Viewer({
        divID : this.divID,
        width : 800,
        height : 600,
        antialias : true,
        fixedFrame: 'panda_link0'
      });

      // Add a grid.
      viewer.addObject(new ROS3D.Grid());

      // Setup a client to listen to TFs.
      let tfClient = new ROSLIB.TFClient({
        fixedFrame: 'panda_link0',
        ros : this.ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
      });

      // Setup the URDF client.
      let urdfClient = new ROS3D.UrdfClient({
        ros : this.ros,
        tfClient : tfClient,
        // urdfModel: 'panda_arm_no_hand.urdf',
        path : 'https://raw.githubusercontent.com/Wisc-HCI/panda-primitives/refs/heads/master/src/authoring/', // TODO: Host these somewhere else???
        rootObject : viewer.selectableObjects,
        loader : ROS3D.COLLADA_LOADER_2
      });
    }

  }
