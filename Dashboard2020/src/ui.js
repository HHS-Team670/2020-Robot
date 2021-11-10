var date = new Date();
var PopupClass = require('js-popup');
var fs = require('fs');

var paths;

document.getElementById('big-warning').style.display = "none";
// document.getElementById('auton-chooser').style.display = "none";



function buildAutonChooser() {
    // get paths
    fetch("http://10.6.70.26:5800/paths").then((result) => {
        result.json().then((pathres) => {
            paths = pathres;
            let startPositions = [];
            let endPositions = [];

            let startPositionsSelection = document.getElementById("start_position");
            let endPositionsSelection = document.getElementById("end_position");

            if (paths) {
                for (let path of paths) {
                if (startPositions.indexOf(path.start_position) < 0) {
                    startPositions.push(path.start_position);
                    let option = new Option(path.start_position, path.start_position);
                    startPositionsSelection.appendChild(option);
                }
                if (endPositions.indexOf(path.end_position) < 0) {
                    endPositions.push(path.end_position);
                    let option = new Option(path.end_position, path.end_position);
                    endPositionsSelection.appendChild(option);
                }
                }
            }
            console.log(startPositions);
            console.log(endPositions);
        }).catch((err) => {console.log(err)});
    });
}

// initial camera settings
var driveReversed = false;
var allKeysPressed = new Array();

// listens for robot-state and updates status lights and auton chooser accordingly
NetworkTables.addKeyListener('/SmartDashboard/robot-state', (key, value) => {
    if (value === "autonomousInit()" || value === "disabledPeriodic()") {
        document.getElementById('auton-chooser').style.display = "none";
    } else if (value === "autonomousPeriodic()") {
        document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
        document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
    } else if (value === "teleopInit()" || value === "teleopPeriodic()") {
        document.getElementById('auton-status').style.fill = "none";
        document.getElementById('auton-status').style.stroke = "rgb(255,255,255)";
    }
});

NetworkTables.addKeyListener('/SmartDashboard/paths', (_, value) => {
    if(value) {
        console.log(value);
    }
});

// listens for warnings
NetworkTables.addKeyListener('/SmartDashboard/warnings', (key, value) => {
    document.getElementById('big-warning').style.display = "inline";
    document.getElementById('warnings').innerHTML += (value + "\n");

    setTimeout(() => { document.getElementById('big-warning').style.display = "none"; }, 1000);
    var timeSinceWarningFlashed = Date.getTime();
});

// NetworkTables.addGlobalListener((key, value) => {
//     console.log(key + ": " + value);
// })

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/Balls', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    document.getElementById('balls-in-robot').textContent = 'Balls in Robot: ' + value;
});

// updates vision frame
NetworkTables.addKeyListener('/SmartDashboard/vision-frame-updated', (key, value) => {
    if (value == true) {
        document.getElementById('vision-frame').src = document.getElementById('vision-frame').src;
        NetworkTables.putValue('vision-frame-updated', false);
    }
});

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/DriveBase', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('drivebase-status').style.fill = "rgb(0,255,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('drivebase-status').style.fill = "rgb(255,255,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('drivebase-status').style.fill = "rgb(255,0,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for turret
NetworkTables.addKeyListener('/SmartDashboard/Turret', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('turret-status').style.fill = "rgb(0,255,0)";
        document.getElementById('turret-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'RED') {
        document.getElementById('turret-status').style.fill = "rgb(255,0,0)";
        document.getElementById('turret-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for indexer
NetworkTables.addKeyListener('/SmartDashboard/Indexer', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('indexer-status').style.fill = "rgb(0,255,0)";
        document.getElementById('indexer-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('shooter-status').style.fill = "rgb(255,255,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('indexer-status').style.fill = "rgb(255,0,0)";
        document.getElementById('indexer-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for shooter
NetworkTables.addKeyListener('/SmartDashboard/Shooter', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('shooter-status').style.fill = "rgb(0,255,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('shooter-status').style.fill = "rgb(255,255,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('shooter-status').style.fill = "rgb(255,0,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for Wheel Of Fortune
NetworkTables.addKeyListener('/SmartDashboard/ColorWheelSpinner', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('wof-status').style.fill = "rgb(0,255,0)";
        document.getElementById('wof-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('wof-status').style.fill = "rgb(255,255,0)";
        document.getElementById('wof-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('wof-status').style.fill = "rgb(255,0,0)";
        document.getElementById('wof-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for Climber
NetworkTables.addKeyListener('/SmartDashboard/Climber', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('climb-status').style.fill = "rgb(0,255,0)";
        document.getElementById('climb-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('climb-status').style.fill = "rgb(255,255,0)";
        document.getElementById('climb-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('climb-status').style.fill = "rgb(255,0,0)";
        document.getElementById('climb-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for Intake
NetworkTables.addKeyListener('/SmartDashboard/Intake', (key, value) => {
    var statusLights = document.getElementById('status-lights-commands');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('intake-status').style.fill = "rgb(0,255,0)";
        document.getElementById('intake-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('intake-status').style.fill = "rgb(255,255,0)";
        document.getElementById('intake-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('intake-status').style.fill = "rgb(255,0,0)";
        document.getElementById('intake-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for vision
NetworkTables.addKeyListener('/Vision/vision-data', (key, value) => {
    var statusLights = document.getElementById('status-lights-commands');
    var commands = statusLights.contentDocument;
    if (value === 'engaged') {
        document.getElementById('vision-status').style.fill = "rgb(0,255,0)";
        document.getElementById('vision-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'invalid-target') {
        document.getElementById('vision-status').style.fill = "rgb(241,244,66)";
        document.getElementById('vision-status').style.stroke = "rgb(241,244,66)";
    } else if (value === 'error' || value === -99999) {
        document.getElementById('vision-status').style.fill = "rgb(255,0,0)";
        document.getElementById('vision-status').style.stroke = "rgb(255,0,0)";
    } else {
        document.getElementById('vision-status').style.fill = "none";
        document.getElementById('vision-status').style.stroke = "rgb(255,255,255)";
    }
});

// document.getElementById("").onclick = function() {
//   sendAuton();
// };

// listens for keystrokes from the external keypad and passes the corresponding values over networktables
var keys = [];
var allKeys = '';
document.addEventListener("keyup", function(event) {
    var pressed = event.key.replace("Enter", "");
    allKeys += pressed;
    var result = allKeys[allKeys.length - 1];
    var nextTask = getFromMap(result);

    console.log(nextTask);
    allKeysPressed.push(nextTask);

    // make sure the key pressed is a valid action
    if (nextTask != null) {
        if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue('/SmartDashboard/xkeys-robotstates', nextTask);
        else if (nextTask.includes("cancel")) NetworkTables.putValue('/SmartDashboard/xkeys-cancel', nextTask);
        else if (nextTask.includes("shoot")) NetworkTables.putValue('/SmartDashboard/xkeys-shooter', nextTask);
        else if (nextTask.includes("updraw")) NetworkTables.putValue('/SmartDashboard/xkeys-updraw', nextTask);
        else if (nextTask.includes("climb")) NetworkTables.putValue('/SmartDashboard/xkeys-climber', nextTask);
        else if (nextTask.includes("vision")) NetworkTables.putValue('/Vision/vision-data', nextTask);
        else if (nextTask.includes("intake") || nextTask.includes("roller")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask);

    }
});

function getFromMap(key) {

    // public static final double RUN_INTAKE_IN = 0;
    // public static final double RUN_INTAKE_OUT = 1;
    // public static final double TOGGLE_INTAKE = 2;

    // public static final double INIT_SHOOTER = 4;
    // public static final double SHOOT = 6;
    // public static final double SHOOT_ALL = 7;

    // public static final double INCREASE_SHOOTER_RPM = 8;
    // public static final double DECREASE_SHOOTER_RPM = 9;

    // public static final double INDEXER_INTAKE = 10;

    // public static final double EXTEND_CLIMBER = 12;
    // public static final double RETRACT_CLIMBER = 13;

    // public static final double SHOOT_NEAR = 14;
    // public static final double SHOOT_MID = 15;
    // public static final double SHOOT_LONG = 16;

    // public static final double CANCEL_ALL = 18;
    if(key  ===  "l")return "1";
    if (key === "3") return "4";
    if (key === "5") return "6";
    if (key === "2") return "7";
    if (key === "6") return "22";

    if (key === "c") return "14";
    if (key === "j") return "15";
    if (key === "d") return "16";
    if (key === "n") return "21";

    if (key === "k") return "2";
    if (key === "e") return "19";
    if (key === "f") return "20";

    if (key === "a") return "NEUTRAL";

    if (key === "m") return "12";
    if (key === "n") return "13";

    if (key === "y") return "vision";

    if (key === "v") return "18";

    return null;
}

function getAutonFromMap() {

    // LEFT_TO_GENERATOR_2_BALL_SIDE(0),
    // LEFT_TO_GENERATOR_3_BALL_SIDE(1),
    // LEFT_TO_TRENCH(2),
    // LEFT_TO_GENERATOR_2_TO_TRENCH(3),
    // LEFT_TO_GENERATOR_3_TO_2_BALL_SIDE(4),

    // CENTER_TO_GENERATOR_2_BALL_SIDE(5),
    // CENTER_TO_GENERATOR_3_BALL_SIDE(6),
    // CENTER_TO_TRENCH(7),
    // CENTER_TO_GENERATOR_2_TO_TRENCH(8),
    // CENTER_TO_GENERATOR_3_TO_2_BALL_SIDE(9),

    // RIGHT_TO_GENERATOR_2_BALL_SIDE(10),
    // RIGHT_TO_GENERATOR_3_BALL_SIDE(11),
    // RIGHT_TO_TRENCH(12),
    // RIGHT_TO_GENERATOR_2_TO_TRENCH(13),
    // RIGHT_TO_GENERATOR_3_TO_2_BALL_SIDE(14),

    // UNKNOWN(-1);

    // LEFT_EMPTY_THEN_BACK(0),
    // LEFT_EMPTY_THEN_FRONT(1),

    // CENTER_EMPTY_THEN_BACK(2),
    // CENTER_EMPTY_THEN_FRONT(3),

    // RIGHT_EMPTY_THEN_BACK(4),
    // RIGHT_EMPTY_THEN_FRONT(5),

    // RIGHT_TO_TRENCH_SHOT(6),



    // console.log(document.querySelector('input[name="start-position"]:checked').value)
    // switch (document.querySelector('input[name="start-position"]:checked').value) {
    //     case "Left":
    //         return getLocation(0, document.querySelector('input[name="location"]:checked').value)
    //     case "Center":
    //         return getLocation(1, document.querySelector('input[name="location"]:checked').value)
    //     case "Right":
    //         return getLocation(2, document.querySelector('input[name="location"]:checked').value)
    // }

    // console.log(document.getElementById("position").value);
}

function getLocation(offset, value) {
    switch (value) {
        case "Generator 2":
            return 0 + offset;
        case "Generator 3":
            return 3 + offset;
        case "Trench":
            return 6 + offset;
    }
    return -1;
}


function sendAuton()  {
    let selectedStartPosition = document.getElementById('start_position').value;
    let selectedEndPosition = document.getElementById('end_position').value;
    let selectedThroughTrench = document.getElementById("through_trench").checked;

    let matchingPath;
    paths.forEach(element => {
        if(element.start_position == selectedStartPosition && element.end_position == selectedEndPosition && element.through_trench == selectedThroughTrench) {
            console.log("Match found " + element.id)
            console.table(element);
            matchingPath = element;
        }
    });
    if(matchingPath) {
        console.log(`Found match ${matchingPath.name}: ${matchingPath.id}`)
        NetworkTables.putValue('/SmartDashboard/auton-chooser', matchingPath.id);
    } else {
        alert("Error: There is no path that matches the contrtains given! Try adjusting your path config")
    }
    
}

buildAutonChooser();