import { Injectable } from '@angular/core';
import { BehaviorSubject, Observable } from 'rxjs';
import '../../../assets/roslib.js';

@Injectable({
    providedIn: 'root'
})

export class CurrentService {
    // Creates object with the ROS library
    // @ts-ignore <= Makes ts happy, wont error
    ros = new ROSLIB.Ros({
        // Set listen URL for ROS communication
        url : 'ws://master:9090'
    });
    // Initialize variables to hold ROS data
    current: BehaviorSubject<any> = new BehaviorSubject('Untouched');

    // Initialize function sets everything up, called on a ngOnInit in app.component

    initialize() {
        // Get Data from ROS Drive Control Listener Topic
        // @ts-ignore
        const currentListener = new ROSLIB.Topic({
            ros : this.ros, // Points to ROS variable
            name : '/rov/converter1/current', // Topic Name
            messageType : 'std_msgs/Float64' // Message Type
        });

        // Subscribe to ROS data
        currentListener.subscribe((message) => {
            // Adds next voltage to pass through to observable driveControl
            this.current.next(message);
        });
    }

    // Getters
    // Get data variable and return it as observable
    getData(): Observable<any> { return this.current.asObservable(); }
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

// Data is gotten through subscription
// this.driveControlService.getDriveControlData().subscribe(msg => {
//   this.data = msg;
//   console.log(msg);
// });
