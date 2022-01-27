var date = new Date();
var PopupClass = require('js-popup');
var runtimer = false;

document.getElementById('big-warning').style.display = "none";
// document.getElementById('auton-chooser').style.display = "none";

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

// listens for warnings
NetworkTables.addKeyListener('/SmartDashboard/warnings', (key, value) => {
    document.getElementById('big-warning').style.display = "inline";
    document.getElementById('warnings').innerHTML += (value + "\n");

    setTimeout(() => { document.getElementById('big-warning').style.display = "none"; }, 1000);
    var timeSinceWarningFlashed = Date.getTime();
});

NetworkTables.addGlobalListener((key, value) => {
    console.log(key + ": " + value);
})

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/Balls', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    document.getElementById('balls-in-robot').textContent = 'Balls in Robot: ' + value;
});

// TODO: add a timer widget thing
// Set the date we're counting down to


var countDownTimer;
var countDownDate;
const AUTON_TIME_MILLIS = timeToMillis("0:05");
const MATCH_LENGTH_MILLIS = timeToMillis("0:10");
var matchPhase = document.getElementById("match-phase");
var timer = document.getElementById("timer");
var timerPrefixString = "Time of Match: ";
var phasePrefixString = "Match Phase: ";
var timeoutFunc = null;
const MatchPhases = Object.freeze ({
    NOT_STARTED: Object.freeze( {text: "NOT STARTED", color:"rgb(50,50,50)"}),
    AUTON: Object.freeze({text: "AUTON", color:"rgb(70,70,200)"}),
    TELEOP: Object.freeze({text:"TELEOP",color:"rgb(0,200,0)"}),
    ENDED: Object.freeze({text:"ENDED",color:"rgb(200,0,0)"})
});


function timeToMillis (timeString) {
    let arr = timeString.split(":");
    let min = parseFloat(arr[0]);
    let sec = parseFloat(arr[1]);
    return (60*min+sec)*1000;
}

function stopTimer () {
    if (countDownTimer != null) clearInterval(countDownTimer);
    timer.textContent = timerPrefixString;
    runtimer = false;
}

function setMatchPhase(phase) {
    
    matchPhase.textContent = phasePrefixString + phase.text;
    matchPhase.style.backgroundColor = phase.color;
}

document.getElementById("timer-stopper").onmouseup = function() {
    stopTimer();
    setMatchPhase(MatchPhases.NOT_STARTED);
}

document.getElementById("timer-starter").onmouseup = function() {
    if (runtimer) return;
    runtimer = true;
    countDownDate = new Date().getTime() + MATCH_LENGTH_MILLIS;
    
    
    timer.textContent = timerPrefixString;
    if (timeoutFunc != null) clearTimeout(timeoutFunc);
    setMatchPhase(MatchPhases.AUTON);
    
    countDownTimer = setInterval(function() {
        
        
        var now = new Date().getTime();
            
        var timeDifference = countDownDate - now;
        var timeDifferenceInSeconds = timeDifference / 1000;

        var seconds =( timeDifferenceInSeconds % 60);
        var minutes = Math.floor( ( timeDifferenceInSeconds % (60*60)) / 60);

        // TODO regex format the time
        timer.textContent = timerPrefixString + (seconds == 60 ? minutes + 1 : minutes) + ':' 
            + (Math.round(seconds) < 10 ? "0" : "") 
                + (seconds == 60 ? "00" : Math.round(seconds));
            
        // If the count down is over, write some text 
        if (timeDifference < 0) {
            stopTimer();
            setMatchPhase(MatchPhases.ENDED);
            timeoutFunc = setTimeout(() => {
                setMatchPhase(MatchPhases.NOT_STARTED); 
                clearTimeout(timeoutFunc);
                timeoutFunc = null;
            }, 5000);
        } else if (timeDifference <= MATCH_LENGTH_MILLIS - AUTON_TIME_MILLIS) {
            setMatchPhase(MatchPhases.TELEOP);
        }
    }, 1000);
    runtimer = true;
};



// updates vision frame
NetworkTables.addKeyListener('/SmartDashboard/vision-frame-updated', (key, value) => {
    if (value == true) {
        document.getElementById('vision-frame').src = document.getElementById('vision-frame').src;
        NetworkTables.putValue('vision-frame-updated', false);
    }
});

// updates robot sped
NetworkTables.addKeyListener('/SmartDashboard/expectedSpeed', (key, value) => {
    document.getElementById('speed').textContent = 'Speed: ' + value;
});

// updates robot angle and direction
    NetworkTables.addKeyListener('/SmartDashboard/vision-values-', (key, value) => {
        document.getElementById('distance').textContent = 'Distance: ' + value[0];
        document.getElementById('angle').textContent = 'Angle: ' + value[2];
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

document.getElementById("confirm-button").onclick = function() {
  sendAuton();
};

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

    if (key === "3") return "4";
    if (key === "5") return "6";
    if (key === "2") return "7";

    if (key === "c") return "14";
    if (key === "j") return "15";
    if (key === "d") return "16";

    if (key === "k") return "2";
    if (key === "e") return "0";
    if (key === "f") return "1";

    if (key === "a") return "NEUTRAL";

    if (key === "m") return "12";
    if (key === "n") return "13";

    if (key === "y") return "vision";

    if (key === "v") return "18";

    return null;
}

function getAutonFromMap() {
    switch (document.querySelector('input[name="start-position"]:checked').value) {
        case "Left":
            return 0;
        case "Center":
            return 1;
        case "Right":
            return 2;
        // case "Left":
        //     return getLocation(0, document.querySelector('input[name="location"]:checked').value)
        // case "Center":
        //     return getLocation(1, document.querySelector('input[name="location"]:checked').value)
        // case "Right":
        //     return getLocation(2, document.querySelector('input[name="location"]:checked').value)
    }
    return -1;

}

// function getLocation(offset, value) {
//     switch (value) {
//         case "Generator 2":
//             return 0 + offset;
//         case "Generator 3":
//             return 3 + offset;
//         case "Trench":
//             return 6 + offset;
//     }
//     return -1;
// }

function sendAuton() {
    var autonCommand = getAutonFromMap();
    console.log(autonCommand);
    NetworkTables.putValue('/SmartDashboard/auton-chooser', autonCommand);
}