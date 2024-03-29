const express   = require('express')
const app       = express()
const port      = 3000


// Make the public folder accessible
app.use(express.static('public'))


app.get('/', (req, res) => {
    res.sendFile('index.html', {root:'./public'})
})


app.get('/settings', (req, res) => {
    res.sendFile('settings.html', {root:'./public'})
})


app.listen(port, () => {
    console.log(`The app is running on localhost:${port}`)
})