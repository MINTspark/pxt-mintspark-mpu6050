namespace MINTsparkMpu6050 {
    let stopheadingCorrections = false;
    let pidCorrection = 0;
    let turnComplete = false;

    export enum TurnDirection
    {
        Left = 0,
        Right = 1
    }

    //% block="Start PID Heading Corrections kp %kp ki %ki kd %kd"
    //% group=Drive
    //% inlineInputMode=inline
    //% weight=99
    export function startPIDHeadingCorrections(Kp: number, Ki: number, Kd: number): void {
        control.inBackground(() => {
            stopheadingCorrections = false;
            pidCorrection = 0;
            let startTime = input.runningTime();
            let lastUpdateTime = startTime;
            let pidController = new MINTsparkMpu6050.PIDController();
            pidController.setGains(Kp, Ki, Kd);
            pidController.setPoint(MINTsparkMpu6050.UpdateMPU6050().orientation.yaw);

            while (!stopheadingCorrections) {
                let updateTime = input.runningTime();
                pidCorrection = pidController.compute(updateTime - lastUpdateTime, UpdateMPU6050().orientation.yaw);
                lastUpdateTime = updateTime;
                basic.pause(10);
            }
        });
    }

    //% block="Stop PID Heading Corrections"
    //% group=Drive
    //% inlineInputMode=inline
    //% weight=98
    export function stopPIDHeadingCorrections(): void {
        stopheadingCorrections = true;
    }

    //% block="Heading Correction"
    //% group=Drive
    //% inlineInputMode=inline
    //% weight=97
    export function getCurrentCorrection(): number {
        return pidCorrection;
    }

    //% block="Start turn monitoring %direction angle %targetAngle"
    //% group=Drive
    //% inlineInputMode=inline
    //% weight=89
    export function startTurn(direction: TurnDirection,targetAngle: number): void {
        turnComplete = false;
        control.inBackground(() => {             
            let startTime = input.runningTime();
            let startHeading = MINTsparkMpu6050.UpdateMPU6050().orientation.yaw;
            let previousHeading = startHeading;
            let totalChange = 0;
            let heading = 0;
            let change = 0;
            basic.pause(50)

            while (input.runningTime() - startTime < 5000) {
                heading = MINTsparkMpu6050.UpdateMPU6050().orientation.yaw;
                
                change = previousHeading - heading;

                let change1 = change;
                if (direction == TurnDirection.Right) {
                    change *= -1;
                }
                if (change < 0) {
                    change += 360
                }
                totalChange += change

       datalogger.log(
           datalogger.createCV("angle", targetAngle),
         datalogger.createCV("startHeading", startHeading),
         datalogger.createCV("heading", heading),
        datalogger.createCV("change", change),
           datalogger.createCV("change1", change1),
           datalogger.createCV("total", totalChange)
         )

                if (totalChange > targetAngle) {
                    music.play(music.tonePlayable(262, music.beat(BeatFraction.Eighth)), music.PlaybackMode.InBackground)
                    break;
                }
                previousHeading = heading
                basic.pause(10)
            }

            turnComplete = true;

        });
    }

    //% block="Turn Complete"
    //% group=Drive
    //% inlineInputMode=inline
    //% weight=87
    export function isTurnComplete(): boolean {
        return turnComplete;
    }
}