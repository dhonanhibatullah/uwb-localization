// const ulsanchor = require('./uwblocsim_modules/uwblocsim_anchor')
// const ulstag    = require('./uwblocsim_modules/uwblocsim_tag')
// const ulscenter = require('./uwblocsim_modules/uwblocsim_center')
// const ulsmap    = require('./uwblocsim_modules/uwblocsim_map')

// class ULSSim {

//     constructor(canvas_id, canvas_width, canvas_height, anchors, tags, map) {
        
//         this.canvas_id      = canvas_id
//         this.canvas_width   = canvas_width
//         this.canvas_height  = canvas_height
//     }


// }

// const canvas = document.getElementById('locMap');
// canvas.width = 400;
// canvas.height = 400;

// const ctx = canvas.getContext('2d');

// function Circle (x, y, r, c) {
//  this.x = x;
//  this.y = y;
//  this.r = r;
//  this.c = c;
 
//  this.dx = Math.floor(Math.random() * 4) + 1;
//  this.dx *= Math.floor(Math.random() * 2) == 1 ? 1 : -1;
//  this.dy = Math.floor(Math.random() * 4) + 1;
//  this.dy *= Math.floor(Math.random() * 2) == 1 ? 1 : -1;
 
//  this.draw = function () {
//   ctx.beginPath();
//   ctx.fillStyle = this.c;
//   ctx.arc(this.x, this.y, this.r, 0, Math.PI * 2);
//   ctx.fill();
//  }
 
//  this.animate = function () {
//   this.x += this.dx;
//   this.y += this.dy;
  
//   if (this.x + this.r > canvas.width || this.x - this.r < 0) {
//    this.dx = -this.dx;
//   }
  
//   if (this.y + this.r > canvas.height || this.y - this.r < 0) {
//    this.dy = -this.dy;
//   }
  
//   this.draw();
//  }
// }

// const balls = [];
// for (let i = 0; i < 20; i++) {
//  let r = Math.floor(Math.random() * 30) + 15;
//  let x = Math.random() * (canvas.width - r * 2) + r;
//  let y = Math.random() * (canvas.height - r * 2) + r;
//  let c = 'red';
//  balls.push(new Circle(x, y, r, c));
// }
// /* BONUS FEATURE */
// canvas.addEventListener('click', function (e) {
//  let r = Math.floor(Math.random() * 30) + 15;
//  balls.push(new Circle(e.clientX, e.clientY, r, 'blue'));
// });

// function Update () {
//  ctx.clearRect(0, 0, canvas.width, canvas.height);
 
//  for (let i = 0; i < balls.length; i++) {
//   let ball = balls[i];
//   ball.animate();
//  }
 
//  requestAnimationFrame(Update);
// }
// Update();