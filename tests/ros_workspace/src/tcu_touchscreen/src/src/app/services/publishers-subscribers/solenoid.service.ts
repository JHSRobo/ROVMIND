import { Injectable } from '@angular/core';
import '../../../assets/roslib.js';
import {BehaviorSubject, Observable} from "rxjs";

@Injectable({
  providedIn: 'root'
})
export class SolenoidService{

  // Creates object with the ROS library
    // @ts-ignore <= Makes ts happy, wont error
    ros = new ROSLIB.Ros({
        // Set listen URL for ROS communication
        url : 'ws://master:9090'
    });
    // Initialize variables to hold ROS data
    solenoidTopic;

    // Initialize subscriber variable to hold data
    solenoidSubscriber: BehaviorSubject<any> = new BehaviorSubject('Untouched');


    // Initialize function sets everything up, called on a ngOnInit in app.component

    initialize() {
        // Initialize ROS topic
        // @ts-ignore
        this.solenoidTopic = new ROSLIB.Topic({
            ros : this.ros, // Points to ROS variable
            name : '/tcu/main_solenoid', // Topic Name
            messageType : 'std_msgs/Bool' // Message Type
        });

        // Subscribe to ROS data
        this.solenoidTopic.subscribe((message) => {
            // Adds next safety value (true/false) to pass through to observable safety
            this.solenoidSubscriber.next(message);
        });
    }

    // publish data that's passed through
    publish(status: Boolean) {
        // @ts-ignore
        const message = new ROSLIB.Message({
            data: status
        });
        this.solenoidTopic.publish(message);
    }

    // Getters
    // Get data variable and return it as observable
    getData(): Observable<any> {
        return this.solenoidSubscriber.asObservable();
    }
}
