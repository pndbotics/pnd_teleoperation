// Global state
let isKeyboardEnabled = false;
let isRobotEngaged = false;
let currentConfig = {};


// Check if running in VR/AR mode
function isVRMode() {
  return window.navigator.xr && document.fullscreenElement;
}

// Update UI based on device
function updateUIForDevice() {
  const desktopInterface = document.getElementById('desktopInterface');
  const vrContent = document.getElementById('vrContent');
  
  if (isVRMode()) {
    desktopInterface.style.display = 'none';
    vrContent.style.display = 'none';
  } else {
    // Check if this is a VR-capable device
    if (navigator.xr) {
      navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
        if (supported) {
          // VR-capable device - show VR interface
          desktopInterface.style.display = 'none';
          vrContent.style.display = 'block';
        } else {
          // Not VR-capable - show desktop interface
          desktopInterface.style.display = 'block';
          vrContent.style.display = 'none';
        }
      }).catch(() => {
        // Fallback to desktop interface if XR check fails
        desktopInterface.style.display = 'block';
        vrContent.style.display = 'none';
      });
    } else {
      // No XR support - show desktop interface
      desktopInterface.style.display = 'block';
      vrContent.style.display = 'none';
    }
  }
}

// Web-based keyboard control
let pressedKeys = new Set();

// Add keyboard event listeners for web-based control
document.addEventListener('keydown', handleKeyDown);
document.addEventListener('keyup', handleKeyUp);

function handleKeyDown(event) {
  // Prevent default browser behavior for our control keys regardless of keyboard state
  if (isControlKey(event.code)) {
    event.preventDefault();
  }
  
  // Only handle keys if keyboard control is enabled and we're focused on the page
  if (!isKeyboardEnabled || pressedKeys.has(event.code)) return;
  
  if (isControlKey(event.code)) {
    pressedKeys.add(event.code);
    sendKeyCommand(event.code, 'press');
  }
}

function handleKeyUp(event) {
  // Prevent default browser behavior for our control keys regardless of keyboard state
  if (isControlKey(event.code)) {
    event.preventDefault();
  }
  
  // Only handle keys if keyboard control is enabled
  if (!isKeyboardEnabled || !pressedKeys.has(event.code)) return;
  
  if (isControlKey(event.code)) {
    pressedKeys.delete(event.code);
    sendKeyCommand(event.code, 'release');
  }
}

function isControlKey(code) {
  // Check if this is one of our robot control keys
  const controlKeys = [
    // Left arm
    'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 
    'KeyZ', 'KeyX', 'KeyF',
    // Right arm
    'KeyI', 'KeyK', 'KeyJ', 'KeyL', 'KeyU', 'KeyO',
    'KeyN', 'KeyM', 'Semicolon',
    // Global
    'Escape'
  ];
  return controlKeys.includes(code);
}

function sendKeyCommand(keyCode, action) {
  // Convert browser keyCode to our key mapping
  const keyMap = {
    // Left arm
    'KeyW': 'w', 'KeyS': 's', 'KeyA': 'a', 'KeyD': 'd',
    'KeyQ': 'q', 'KeyE': 'e', 'KeyZ': 'z', 'KeyX': 'x',
    'KeyF': 'f',
    // Right arm  
    'KeyI': 'i', 'KeyK': 'k', 'KeyJ': 'j', 'KeyL': 'l',
    'KeyU': 'u', 'KeyO': 'o', 'KeyN': 'n', 'KeyM': 'm',
    'Semicolon': ';',
    // Global
    'Escape': 'esc'
  };

  const key = keyMap[keyCode];
  if (!key) return;
}

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  updateUIForDevice();
  
  // Start status monitoring
  // updateStatus();
  // setInterval(updateStatus, 2000); // Update every 2 seconds
  
  // Handle VR mode changes
  document.addEventListener('fullscreenchange', updateUIForDevice);
  
  // VR session detection
  if (navigator.xr) {
    navigator.xr.addEventListener('sessionstart', () => {
      // updateStatus();
      updateUIForDevice();
    });
    
    navigator.xr.addEventListener('sessionend', () => {
      // updateStatus();
      updateUIForDevice();
    });
  }

});

// Handle window resize
// window.addEventListener('resize', updateUIForDevice); 