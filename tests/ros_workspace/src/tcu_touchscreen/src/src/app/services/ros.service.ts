import { Injectable } from '@angular/core';

@Injectable({
    providedIn: 'root'
})
export class RosService {

    // Creates object with the ROS library
    // @ts-ignore <= Makes ts happy, wont error
    ros = new ROSLIB.Ros({
        // Set listen URL for ROS communication
        url : 'ws://master:9090'
    });

    initialize() {
        // Listens for error from ROS and logs it
        this.ros.on('error', function(error) {
            console.log(error);
        });

        // Find out exactly when we made a connection.
        this.ros.on('connection', function() {
            console.log('Connection made!');
        });
        // Logs when connection is closed
        this.ros.on('close', function() {
            console.log('Connection closed.');
        });
    }
}

// Data is gotten through subscription from each node service
// this.driveControlService.getDriveControlData().subscribe(msg => {
//   this.data = msg;
//   console.log(msg);
// });
// }
