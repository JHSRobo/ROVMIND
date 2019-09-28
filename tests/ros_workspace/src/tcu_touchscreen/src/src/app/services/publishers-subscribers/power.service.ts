import { Injectable } from '@angular/core';
import '../../../assets/roslib.js';
import {BehaviorSubject, Observable} from "rxjs";

@Injectable({
  providedIn: 'root'
})
export class PowerService {

    // Creates object with the ROS library
    // @ts-ignore <= Makes ts happy, wont error
    ros = new ROSLIB.Ros({
        // Set listen URL for ROS communication
        url : 'ws://master:9090'
    });

    // Initialize variable to hold topic
    powerTopic;

    // Initialize variables to hold ROS data
    powerSubscriber: BehaviorSubject<any> = new BehaviorSubject('Untouched');

    // Initialize function sets everything up, called on a ngOnInit in app.component

    initialize() {
        // Initialize ROS topic
        // @ts-ignore
        this.powerTopic = new ROSLIB.Topic({
            ros : this.ros, // Points to ROS variable
            name : '/tcu/main_relay', // Topic Name
            messageType : 'std_msgs/Bool' // Message Type
        });

        // Subscribe to ROS data
        this.powerTopic.subscribe((message) => {
            // Adds next voltage to pass through to observable driveControl
            this.powerSubscriber.next(message);
        });
    }

    // publish data that's passed through
    publish(status: boolean) {
        console.log('Thruster Service Called');
        console.log(status);
        // @ts-ignore
        const message = new ROSLIB.Message({
            data: status
        });
        this.powerTopic.publish(message);
    }

    getData(): Observable<any> {
        return this.powerSubscriber.asObservable();
    }
}
