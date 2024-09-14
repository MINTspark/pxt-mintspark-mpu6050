namespace MINTsparkMpu6050 {
    let stopheadingCorrections = false;
    let pidCorrection = 0;

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
}