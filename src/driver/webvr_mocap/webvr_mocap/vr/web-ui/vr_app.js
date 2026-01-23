// Wait for A-Frame scene to load

AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log("Controller updater component initialized.");
    // Controllers are enabled
    this.initElements();
    this.initWebSocket();
    this.initState();
  },

  initElements: function () {
    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');

    // Add headset tracking
    this.headset = document.querySelector('#headset');
    this.headsetInfoText = document.querySelector('#headsetInfo');

    if (!this.leftHand || !this.rightHand || !this.leftHandInfoText || !this.rightHandInfoText) {
      console.error("Controller or text entities not found!");
      // Check which specific elements are missing
      if (!this.leftHand) console.error("Left hand entity not found");
      if (!this.rightHand) console.error("Right hand entity not found");
      if (!this.leftHandInfoText) console.error("Left hand info text not found");
      if (!this.rightHandInfoText) console.error("Right hand info text not found");
      return;
    }

    // Apply initial rotation to combined text elements
    const textRotation = '-90 0 0'; // Rotate -90 degrees around X-axis
    if (this.leftHandInfoText) this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText) this.rightHandInfoText.setAttribute('rotation', textRotation);

  },

  initWebSocket: function () {
    // --- WebSocket Setup ---
    // --- Get hostname dynamically ---
    const serverHostname = window.location.hostname;
    const websocketPort = 8442; // Make sure this matches controller_server.py
    const websocketUrl = `wss://${serverHostname}:${websocketPort}`;
    console.log(`Attempting WebSocket connection to: ${websocketUrl}`);
    // !!! IMPORTANT: Replace 'YOUR_LAPTOP_IP' with the actual IP address of your laptop !!!
    // const websocketUrl = 'ws://YOUR_LAPTOP_IP:8442';
    try {
      this.websocket = new WebSocket(websocketUrl);
      this.websocket.onopen = (event) => {
        console.log(`WebSocket connected to ${websocketUrl}`);
        this.reportVRStatus(true);
      };
      this.websocket.onerror = (event) => {
        // More detailed error logging
        console.error(`WebSocket Error: Event type: ${event.type}`, event);
        this.reportVRStatus(false);
      };
      this.websocket.onclose = (event) => {
        console.log(`WebSocket disconnected from ${websocketUrl}. Clean close: ${event.wasClean}, Code: ${event.code}, Reason: '${event.reason}'`);
        // Attempt to log specific error if available (might be limited by browser security)
        if (!event.wasClean) {
          console.error('WebSocket closed unexpectedly.');
        }
        this.websocket = null; // Clear the reference
        this.reportVRStatus(false);
      };
      this.websocket.onmessage = (event) => {
        console.log(`WebSocket message received: ${event.data}`); // Log any messages from server
      };
    } catch (error) {
      console.error(`Failed to create WebSocket connection to ${websocketUrl}:`, error);
      this.reportVRStatus(false);
    }
    // --- End WebSocket Setup ---

    // --- VR Status Reporting Function ---
    this.reportVRStatus = (connected) => {
      // Update global status if available (for desktop interface)
      if (typeof updateStatus === 'function') {
        updateStatus({ vrConnected: connected });
      }

      // Also try to notify parent window if in iframe
      try {
        if (window.parent && window.parent !== window) {
          window.parent.postMessage({
            type: 'vr_status',
            connected: connected
          }, '*');
        }
      } catch (e) {
        // Ignore cross-origin errors
      }
    };

    this.sendWebSocketData = (data) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        this.websocket.send(JSON.stringify(data));
      }
    }
  },

  initState: function () {
    this.leftGripDown = false;
    this.rightGripDown = false;

    // --- Relative rotation tracking ---
    this.leftGripInitialRotation = null;
    this.rightGripInitialRotation = null;
    this.leftRelativeRotation = { x: 0, y: 0, z: 0 };
    this.rightRelativeRotation = { x: 0, y: 0, z: 0 };

    // --- Quaternion-based Z-axis rotation tracking ---
    this.leftGripInitialQuaternion = null;
    this.rightGripInitialQuaternion = null;
    this.leftZAxisRotation = 0;
    this.rightZAxisRotation = 0;

    // --- Create axis indicators ---
    this.createAxisIndicators();

    // --- Helper function to calculate relative rotation ---
    this.calculateRelativeRotation = (currentRotation, initialRotation) => {
      return {
        x: currentRotation.x - initialRotation.x,
        y: currentRotation.y - initialRotation.y,
        z: currentRotation.z - initialRotation.z
      };
    };

    // --- Helper function to calculate Z-axis rotation from quaternions ---
    this.calculateZAxisRotation = (currentQuaternion, initialQuaternion) => {
      // Calculate relative quaternion (from initial to current)
      const relativeQuat = new THREE.Quaternion();
      relativeQuat.multiplyQuaternions(currentQuaternion, initialQuaternion.clone().invert());

      // Get the controller's current forward direction (local Z-axis in world space)
      const forwardDirection = new THREE.Vector3(0, 0, 1);
      forwardDirection.applyQuaternion(currentQuaternion);

      // Convert relative quaternion to axis-angle representation
      const angle = 2 * Math.acos(Math.abs(relativeQuat.w));

      // Handle case where there's no rotation (avoid division by zero)
      if (angle < 0.0001) {
        return 0;
      }

      // Get the rotation axis
      const sinHalfAngle = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
      const rotationAxis = new THREE.Vector3(
        relativeQuat.x / sinHalfAngle,
        relativeQuat.y / sinHalfAngle,
        relativeQuat.z / sinHalfAngle
      );

      // Project the rotation axis onto the forward direction to get the component
      // of rotation around the forward axis
      const projectedComponent = rotationAxis.dot(forwardDirection);

      // The rotation around the forward axis is the angle times the projection
      const forwardRotation = angle * projectedComponent;

      // Convert to degrees and handle the sign properly
      let degrees = THREE.MathUtils.radToDeg(forwardRotation);

      // Normalize to -180 to +180 range to avoid sudden jumps
      while (degrees > 180) degrees -= 360;
      while (degrees < -180) degrees += 360;

      return degrees;
    };

    // --- Modify Event Listeners ---
    this.leftHand.addEventListener('gripdown', (evt) => {
      console.log('Left Grip Pressed');
      this.leftGripDown = true; // Set grip state

      // Store initial rotation for relative tracking
      if (this.leftHand.object3D.visible) {
        const leftRotEuler = this.leftHand.object3D.rotation;
        this.leftGripInitialRotation = {
          x: THREE.MathUtils.radToDeg(leftRotEuler.x),
          y: THREE.MathUtils.radToDeg(leftRotEuler.y),
          z: THREE.MathUtils.radToDeg(leftRotEuler.z)
        };

        // Store initial quaternion for Z-axis rotation tracking
        this.leftGripInitialQuaternion = this.leftHand.object3D.quaternion.clone();

        console.log('Left grip initial rotation:', this.leftGripInitialRotation);
        console.log('Left grip initial quaternion:', this.leftGripInitialQuaternion);
      }
    });
    this.leftHand.addEventListener('gripup', (evt) => { // Add gripup listener
      console.log('Left Grip Released');
      this.leftGripDown = false; // Reset grip state
      this.leftGripInitialRotation = null; // Reset initial rotation
      this.leftGripInitialQuaternion = null; // Reset initial quaternion
      this.leftRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
      this.leftZAxisRotation = 0; // Reset Z-axis rotation
    });

    this.rightHand.addEventListener('gripdown', (evt) => {
      console.log('Right Grip Pressed');
      this.rightGripDown = true; // Set grip state

      // Store initial rotation for relative tracking
      if (this.rightHand.object3D.visible) {
        const rightRotEuler = this.rightHand.object3D.rotation;
        this.rightGripInitialRotation = {
          x: THREE.MathUtils.radToDeg(rightRotEuler.x),
          y: THREE.MathUtils.radToDeg(rightRotEuler.y),
          z: THREE.MathUtils.radToDeg(rightRotEuler.z)
        };

        // Store initial quaternion for Z-axis rotation tracking
        this.rightGripInitialQuaternion = this.rightHand.object3D.quaternion.clone();

        console.log('Right grip initial rotation:', this.rightGripInitialRotation);
        console.log('Right grip initial quaternion:', this.rightGripInitialQuaternion);
      }
    });
    this.rightHand.addEventListener('gripup', (evt) => { // Add gripup listener
      console.log('Right Grip Released');
      this.rightGripDown = false; // Reset grip state
      this.rightGripInitialRotation = null; // Reset initial rotation
      this.rightGripInitialQuaternion = null; // Reset initial quaternion
      this.rightRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
      this.rightZAxisRotation = 0; // Reset Z-axis rotation
    });
    // --- End Modify Event Listeners ---
  },

  createAxisIndicators: function () {
    // Create XYZ axis indicators for both controllers

    // Left Controller Axes
    // X-axis (Red)
    const leftXAxis = document.createElement('a-cylinder');
    leftXAxis.setAttribute('id', 'leftXAxis');
    leftXAxis.setAttribute('height', '0.08');
    leftXAxis.setAttribute('radius', '0.003');
    leftXAxis.setAttribute('color', '#ff0000'); // Red for X
    leftXAxis.setAttribute('position', '0.04 0 0');
    leftXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.leftHand.appendChild(leftXAxis);

    const leftXTip = document.createElement('a-cone');
    leftXTip.setAttribute('height', '0.015');
    leftXTip.setAttribute('radius-bottom', '0.008');
    leftXTip.setAttribute('radius-top', '0');
    leftXTip.setAttribute('color', '#ff0000');
    leftXTip.setAttribute('position', '0.055 0 0');
    leftXTip.setAttribute('rotation', '0 0 90');
    this.leftHand.appendChild(leftXTip);

    // Y-axis (Green) - Up
    const leftYAxis = document.createElement('a-cylinder');
    leftYAxis.setAttribute('id', 'leftYAxis');
    leftYAxis.setAttribute('height', '0.08');
    leftYAxis.setAttribute('radius', '0.003');
    leftYAxis.setAttribute('color', '#00ff00'); // Green for Y
    leftYAxis.setAttribute('position', '0 0.04 0');
    leftYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.leftHand.appendChild(leftYAxis);

    const leftYTip = document.createElement('a-cone');
    leftYTip.setAttribute('height', '0.015');
    leftYTip.setAttribute('radius-bottom', '0.008');
    leftYTip.setAttribute('radius-top', '0');
    leftYTip.setAttribute('color', '#00ff00');
    leftYTip.setAttribute('position', '0 0.055 0');
    this.leftHand.appendChild(leftYTip);

    // Z-axis (Blue) - Forward
    const leftZAxis = document.createElement('a-cylinder');
    leftZAxis.setAttribute('id', 'leftZAxis');
    leftZAxis.setAttribute('height', '0.08');
    leftZAxis.setAttribute('radius', '0.003');
    leftZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    leftZAxis.setAttribute('position', '0 0 0.04');
    leftZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.leftHand.appendChild(leftZAxis);

    const leftZTip = document.createElement('a-cone');
    leftZTip.setAttribute('height', '0.015');
    leftZTip.setAttribute('radius-bottom', '0.008');
    leftZTip.setAttribute('radius-top', '0');
    leftZTip.setAttribute('color', '#0000ff');
    leftZTip.setAttribute('position', '0 0 0.055');
    leftZTip.setAttribute('rotation', '90 0 0');
    this.leftHand.appendChild(leftZTip);

    // Right Controller Axes
    // X-axis (Red)
    const rightXAxis = document.createElement('a-cylinder');
    rightXAxis.setAttribute('id', 'rightXAxis');
    rightXAxis.setAttribute('height', '0.08');
    rightXAxis.setAttribute('radius', '0.003');
    rightXAxis.setAttribute('color', '#ff0000'); // Red for X
    rightXAxis.setAttribute('position', '0.04 0 0');
    rightXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.rightHand.appendChild(rightXAxis);

    const rightXTip = document.createElement('a-cone');
    rightXTip.setAttribute('height', '0.015');
    rightXTip.setAttribute('radius-bottom', '0.008');
    rightXTip.setAttribute('radius-top', '0');
    rightXTip.setAttribute('color', '#ff0000');
    rightXTip.setAttribute('position', '0.055 0 0');
    rightXTip.setAttribute('rotation', '0 0 90');
    this.rightHand.appendChild(rightXTip);

    // Y-axis (Green) - Up
    const rightYAxis = document.createElement('a-cylinder');
    rightYAxis.setAttribute('id', 'rightYAxis');
    rightYAxis.setAttribute('height', '0.08');
    rightYAxis.setAttribute('radius', '0.003');
    rightYAxis.setAttribute('color', '#00ff00'); // Green for Y
    rightYAxis.setAttribute('position', '0 0.04 0');
    rightYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.rightHand.appendChild(rightYAxis);

    const rightYTip = document.createElement('a-cone');
    rightYTip.setAttribute('height', '0.015');
    rightYTip.setAttribute('radius-bottom', '0.008');
    rightYTip.setAttribute('radius-top', '0');
    rightYTip.setAttribute('color', '#00ff00');
    rightYTip.setAttribute('position', '0 0.055 0');
    this.rightHand.appendChild(rightYTip);

    // Z-axis (Blue) - Forward
    const rightZAxis = document.createElement('a-cylinder');
    rightZAxis.setAttribute('id', 'rightZAxis');
    rightZAxis.setAttribute('height', '0.08');
    rightZAxis.setAttribute('radius', '0.003');
    rightZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    rightZAxis.setAttribute('position', '0 0 0.04');
    rightZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.rightHand.appendChild(rightZAxis);

    const rightZTip = document.createElement('a-cone');
    rightZTip.setAttribute('height', '0.015');
    rightZTip.setAttribute('radius-bottom', '0.008');
    rightZTip.setAttribute('radius-top', '0');
    rightZTip.setAttribute('color', '#0000ff');
    rightZTip.setAttribute('position', '0 0 0.055');
    rightZTip.setAttribute('rotation', '90 0 0');
    this.rightHand.appendChild(rightZTip);

    console.log('XYZ axis indicators created for both controllers (RGB for XYZ)');
  },

  tick: function () {
    // Update controller text if controllers are visible
    if (!this.leftHand || !this.rightHand || !this.headset) return; // Added safety check

    // Collect data from both controllers
    const leftController = {
      hand: 'left',
      position: null,
      rotation: null,
      quaternion: null,
    };

    const rightController = {
      hand: 'right',
      position: null,
      rotation: null,
      quaternion: null,
    };

    const headset = {
      position: null,
      rotation: null,
      quaternion: null
    };

    const Joy = {
      axes: [],
      buttons: []
    }

    // Update Left Hand Text & Collect Data
    // 移除object3D.visible检查，确保即使控制器不可见也能收集数据
    if (this.leftHand && this.leftHand.object3D) {
      const leftPos = this.leftHand.object3D.position;
      const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
      // Convert to degrees without offset
      const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
      const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
      const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);

      // 添加调试信息
      console.log(`Left Hand - Visible: ${this.leftHand.object3D.visible}, Pos: ${leftPos.x.toFixed(2)},${leftPos.y.toFixed(2)},${leftPos.z.toFixed(2)}`);

      // Calculate relative rotation if grip is held
      if (this.leftGripDown && this.leftGripInitialRotation) {
        this.leftRelativeRotation = this.calculateRelativeRotation(
          { x: leftRotX, y: leftRotY, z: leftRotZ },
          this.leftGripInitialRotation
        );

        // Calculate Z-axis rotation using quaternions
        if (this.leftGripInitialQuaternion) {
          this.leftZAxisRotation = this.calculateZAxisRotation(
            this.leftHand.object3D.quaternion,
            this.leftGripInitialQuaternion
          );
        }

        // console.log('Left relative rotation:', this.leftRelativeRotation);
        // console.log('Left Z-axis rotation:', this.leftZAxisRotation.toFixed(1), 'degrees');
      }

      // Create display text including relative rotation when grip is held
      let combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}\\nRot: ${leftRotX.toFixed(0)} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
      if (this.leftGripDown && this.leftGripInitialRotation) {
        combinedLeftText += `\\nZ-Rot: ${this.leftZAxisRotation.toFixed(1)}°`;
      }

      if (this.leftHandInfoText) {
        this.leftHandInfoText.setAttribute('value', combinedLeftText);
      }

      // Collect left controller data
      leftController.position = { x: leftPos.x, y: leftPos.y, z: leftPos.z };
      leftController.rotation = { x: leftRotX, y: leftRotY, z: leftRotZ };
      leftController.quaternion = {
        x: this.leftHand.object3D.quaternion.x,
        y: this.leftHand.object3D.quaternion.y,
        z: this.leftHand.object3D.quaternion.z,
        w: this.leftHand.object3D.quaternion.w
      };


      // 采集左手柄的摇杆和按钮信息
      if (this.leftHand && this.leftHand.components && this.leftHand.components['tracked-controls']) {
        const leftGamepad = this.leftHand.components['tracked-controls'].controller?.gamepad;
        if (leftGamepad) {
          // leftController.x = leftGamepad.axes[2]
          // leftController.y = leftGamepad.axes[3]
          // leftController.trigger = leftGamepad.buttons[0].value // Trigger value
          // leftController.grip = leftGamepad.buttons[1].value // Grip value
          // leftController.thumbstick = leftGamepad.buttons[3].value // Thumbstick value
          // leftController.X = leftGamepad.buttons[4].value // A
          // leftController.Y = leftGamepad.buttons[5].value // B

          Joy.axes = Joy.axes.concat([leftGamepad.axes[2], leftGamepad.axes[3], leftGamepad.buttons[0].value, leftGamepad.buttons[1].value]);
          const indexs = [3, 4, 5];
          Joy.buttons = Joy.buttons.concat(leftGamepad.buttons.filter((item, i) => indexs.find((v) => v === i)).map(item => [item.value, item.touched]));
        }
      }
    } else {
      console.log('Left hand object not available');
    }

    // Update Right Hand Text & Collect Data
    // 移除object3D.visible检查，确保即使控制器不可见也能收集数据
    if (this.rightHand && this.rightHand.object3D) {
      const rightPos = this.rightHand.object3D.position;
      const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
      // Convert to degrees without offset
      const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
      const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
      const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);

      // 添加调试信息
      // console.log(`Right Hand - Visible: ${this.rightHand.object3D.visible}, Pos: ${rightPos.x.toFixed(2)},${rightPos.y.toFixed(2)},${rightPos.z.toFixed(2)}`);

      // Calculate relative rotation if grip is held
      if (this.rightGripDown && this.rightGripInitialRotation) {
        this.rightRelativeRotation = this.calculateRelativeRotation(
          { x: rightRotX, y: rightRotY, z: rightRotZ },
          this.rightGripInitialRotation
        );

        // Calculate Z-axis rotation using quaternions
        if (this.rightGripInitialQuaternion) {
          this.rightZAxisRotation = this.calculateZAxisRotation(
            this.rightHand.object3D.quaternion,
            this.rightGripInitialQuaternion
          );
        }

        // console.log('Right relative rotation:', this.rightRelativeRotation);
        // console.log('Right Z-axis rotation:', this.rightZAxisRotation.toFixed(1), 'degrees');
      }

      // Create display text including relative rotation when grip is held
      let combinedRightText = `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}\\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(0)} ${rightRotZ.toFixed(0)}`;
      if (this.rightGripDown && this.rightGripInitialRotation) {
        combinedRightText += `\\nZ-Rot: ${this.rightZAxisRotation.toFixed(1)}°`;
      }

      if (this.rightHandInfoText) {
        this.rightHandInfoText.setAttribute('value', combinedRightText);
      }

      // Collect right controller data
      rightController.position = { x: rightPos.x, y: rightPos.y, z: rightPos.z };
      rightController.rotation = { x: rightRotX, y: rightRotY, z: rightRotZ };
      rightController.quaternion = {
        x: this.rightHand.object3D.quaternion.x,
        y: this.rightHand.object3D.quaternion.y,
        z: this.rightHand.object3D.quaternion.z,
        w: this.rightHand.object3D.quaternion.w
      };

      // 采集右手柄的摇杆和按钮信息
      if (this.rightHand && this.rightHand.components && this.rightHand.components['tracked-controls']) {
        const rightGamepad = this.rightHand.components['tracked-controls'].controller?.gamepad;
        if (rightGamepad) {
          // rightController.x = rightGamepad.axes[2]
          // rightController.y = rightGamepad.axes[3]
          // rightController.trigger = rightGamepad.buttons[0].value // Trigger value
          // rightController.grip = rightGamepad.buttons[1].value // Grip value
          // rightController.thumbstick = rightGamepad.buttons[3].value // Thumbstick value
          // rightController.A = rightGamepad.buttons[4].value // A
          // rightController.B = rightGamepad.buttons[5].value // B

          Joy.axes = Joy.axes.concat([rightGamepad.axes[2], rightGamepad.axes[3], rightGamepad.buttons[0].value, rightGamepad.buttons[1].value]);
          const indexs = [3, 4, 5];
          Joy.buttons = Joy.buttons.concat(rightGamepad.buttons.filter((item, i) => indexs.find((v) => v === i)).map(item => [item.value, item.touched]));
        }
      }
    } else {
      console.log('Right hand object not available');
    }

    // Collect headset data
    if (this.headset && this.headset.object3D) {
      const headsetPos = this.headset.object3D.position;
      const headsetRotEuler = this.headset.object3D.rotation;
      const headsetRotX = THREE.MathUtils.radToDeg(headsetRotEuler.x);
      const headsetRotY = THREE.MathUtils.radToDeg(headsetRotEuler.y);
      const headsetRotZ = THREE.MathUtils.radToDeg(headsetRotEuler.z);

      // Update headset info text
      const headsetText = `Pos: ${headsetPos.x.toFixed(2)} ${headsetPos.y.toFixed(2)} ${headsetPos.z.toFixed(2)}\nRot: ${headsetRotX.toFixed(0)} ${headsetRotY.toFixed(0)} ${headsetRotZ.toFixed(0)}`;
      if (this.headsetInfoText) {
        this.headsetInfoText.setAttribute('value', headsetText);
      }

      // Collect headset data
      headset.position = { x: headsetPos.x, y: headsetPos.y, z: headsetPos.z };
      headset.rotation = { x: headsetRotX, y: headsetRotY, z: headsetRotZ };
      headset.quaternion = {
        x: this.headset.object3D.quaternion.x,
        y: this.headset.object3D.quaternion.y,
        z: this.headset.object3D.quaternion.z,
        w: this.headset.object3D.quaternion.w
      };

      // console.log(`Headset - Pos: ${headsetPos.x.toFixed(2)},${headsetPos.y.toFixed(2)},${headsetPos.z.toFixed(2)}`);
    } else {
      console.log('Headset object not available');
    }


    // if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
    //   // 修改发送条件：只要有位置数据就发送，不检查是否为(0,0,0)
    //   const hasValidLeft = leftController.position !== null;
    //   const hasValidRight = rightController.position !== null;
    //   const hasValidHeadset = headset.position !== null;

    //   if (hasValidLeft || hasValidRight || hasValidHeadset) {


    //     // 添加调试信息
    //     console.log('Sending VR data:', {
    //       left: hasValidLeft ? 'valid' : 'invalid',
    //       right: hasValidRight ? 'valid' : 'invalid',
    //       headset: hasValidHeadset ? 'valid' : 'invalid',
    //       leftPos: leftController.position,
    //       rightPos: rightController.position,
    //       headsetPos: headset.position
    //     });
    //   }
    // }

    const dualControllerData = {
      timestamp: Date.now(),
      LeftHand: leftController,
      RightHand: rightController,
      Head: headset,
      Joy
    };
    this.sendWebSocketData(dualControllerData);
  }
});
