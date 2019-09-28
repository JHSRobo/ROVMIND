import { Component, OnInit } from '@angular/core';
import { Options } from 'ng5-slider';
import { VoltageService } from '../services/subscribers/voltage.service';
import { CurrentService } from '../services/subscribers/current.service';

@Component({
  selector: 'app-sliders',
  templateUrl: './sliders.component.html',
  styleUrls: ['./sliders.component.css']
})
export class SlidersComponent implements OnInit {

    rovVoltage: number; // ROV temperature in Celsius
    rovCurrent: number; // ROV Altitude in atm (can change to pascal)
    maxVoltage = 0;
    maxCurrent = 0;

    options: Options = {
        floor: 0,
        ceil: 40
    };

    options2: Options = {
        floor: 0,
        ceil: 50
    };

    constructor(private VoltageService: VoltageService, private CurrentService: CurrentService) {}

    ngOnInit() {
        this.VoltageService.initialize();
        this.VoltageService.getData().subscribe((msg: Float64) => {
            console.log(msg);
            this.rovVoltage = msg.data;
            if (msg.data > this.maxVoltage) {
                this.maxVoltage = msg.data;
            }
        });
        this.CurrentService.initialize();
        this.CurrentService.getData().subscribe((msg: Float64) => {
            this.rovCurrent = msg.data;
            if (msg.data > this.maxCurrent) {
                this.maxCurrent = msg.data;
            }
        });
    }
}

class Float64 {
    data: number;
}
