const fs        = require('fs')
const path      = require('path')
const jsyaml    = require('js-yaml')
const rclnodejs = require('rclnodejs')


class RclGateAway {

    constructor() {

    }


    nodeInit() {
        rclnodejs.init().then(() => {

            const node = new rclnodejs.Node(`${this.robot_info.robot_id}_WebAppServerNode`)

            node.createSubscription(
                'adisha_interface/msg/JointTorque',
                `${this.robot_info.robot_id}/present_torque`,
                (msg) => {
                    this.response_msg.present_torque = msg.val
                }
            ) 

            const torque_pub = node.createPublisher(
                'adisha_interface/msg/JointTorque',
                `${this.robot_info.robot_id}/goal_torque`
            )

            setInterval(() => {
                if(this.set_torque) {
                    this.set_torque = false
                    torque_pub.publish(this.request_msg.goal_torque)
                }

                if(this.set_pos) {
                    this.set_pos = false
                }

                if(this.set_vel) {
                    this.set_vel = false
                }
            }, 50);
            
            node.spin()
        })
    }
}