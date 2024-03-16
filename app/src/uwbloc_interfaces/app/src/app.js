const path      = require('path')
const express   = require('express')
// const rclga     = require('./js/rcl-gateaway')

const app       = express()
const port      = 3000

app.set('view engine', 'ejs')
app.set('views', path.join(__dirname, '../views'))
app.use(express.static(path.join(__dirname, '../public')))
app.use(express.urlencoded({extended: false}))
app.use(express.json())



app.get('/', (req, res) => {
    res.send("Hello")
})


app.listen(port)