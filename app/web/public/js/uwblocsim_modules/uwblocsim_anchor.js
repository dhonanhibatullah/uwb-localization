class ULSAnchor {

    constructor(id, pos_x, pos_y, noise_amp) {
        
        this.info = {
            id: id,
            pos_x: pos_x,
            pos_y: pos_y,
        }

        this.noise_amp = 0.1
    }

    getTagRange(tag_info) {
        return {
            id: tag_info.id,
            range: Math.sqrt((this.info.pos_x - tag_info.pos_x)**2.0 + (this.info.pos_y - tag_info.pos_y)**2.0) + this.noise_amp*(2.0*Math.random() - 1.0),
        }
    }
}

module.exports = ULSAnchor