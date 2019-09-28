import {Component, OnInit} from '@angular/core';
import { PowerService } from '../services/publishers-subscribers/power.service';
import { SolenoidService } from '../services/publishers-subscribers/solenoid.service';
import { SafetyService } from '../services/publishers-subscribers/safety.service';
import { BooleanModel } from "../services/boolean.model";

@Component({
  selector: 'app-buttons',
  templateUrl: './buttons.component.html',
  styleUrls: ['./buttons.component.css']
})
export class ButtonsComponent implements OnInit{

    constructor(
        public powerService: PowerService,
        public solenoidService: SolenoidService,
        public safetyService: SafetyService) {}

    buttonStyle = 'powerbuttonoff';
    togglework = false;
    togglesafety = false;
    togglepneu = false;
    visible = false;
    power = false;

    openConfirm() {
        this.visible = true;
    }
    close() {
        this.visible = false;
    }

    confirm() {
        this.power = !this.power;
        this.powerService.publish(this.power);
        this.visible = false;
    }

    LightToggle() {
        this.togglework = !this.togglework;
    }

    SafetyToggle() {
        this.togglesafety = !this.togglesafety;
        this.safetyService.publish(this.togglesafety);
    }

    PneumaticsToggle() {
        this.togglepneu = !this.togglepneu;
        this.solenoidService.publish(this.togglepneu);
    }

    ngOnInit() {
        this.powerService.initialize();
        this.solenoidService.initialize();
        this.safetyService.initialize();
        this.powerService.getData().subscribe((msg: BooleanModel) => { this.power = msg.data; });
        this.solenoidService.getData().subscribe((msg: BooleanModel) => { this.togglepneu = msg.data; });
        this.safetyService.getData().subscribe((msg: BooleanModel) => { this.togglesafety = msg.data; });
    }
}
