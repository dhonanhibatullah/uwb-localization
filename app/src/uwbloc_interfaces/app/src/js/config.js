const fs        = require('fs')
const path      = require('path')
const jsyaml    = require('js-yaml')

const CONFIG_PATH       = path.join(__dirname, '../../../config')
const CORE_YAML_PATH    = path.join(CONFIG_PATH, 'core.yaml')

const CORE_YAML = jsyaml.load(fs.readFileSync(CORE_YAML_PATH, 'utf8'))

module.exports.core = CORE_YAML