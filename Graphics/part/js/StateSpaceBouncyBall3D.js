var gl;
var g_canvas;

// conditionals
control0 = true;
control1 = false;
control2 = false;
control3 = false;
control4 = true;
var hasWall = true;
var hasBall = true;

worldBox = new VBObox1();
partBox1 = new VBObox2();
partBox2 = new VBObox3();
partBox4 = new VBObox5();

var g_timeStep = 1000.0/100.0;  // milliseconds
var g_timeStepMin = g_timeStep;
var g_timeStepMax = g_timeStep;
var g_stepCount = 0;

var g_last = Date.now();

var runMode = 3;  // 0: reset; 1: pause; 2: step; 3: run




function main() {
    g_canvas = document.getElementById('webgl');
    gl = g_canvas.getContext("webgl", { preserveDrawingBuffer: true});

    if (!gl) {
        console.log('Failed to get the rendering context for WebGL');
        return;
    }

    worldBox.init(gl);
    partBox1.init(gl);
    partBox2.init(gl);
    partBox4.init(gl);

    // Event register
    window.addEventListener("keydown", myKeyDown, false);
    
    // Dat.gui
    var GUIContent = function() {
        this.solver = 'Explicit-Midpoint';
        this.hasWalls = hasWall;
        this.wallElasticity = Kbouncy;
        this.hascolorfulball = hasBall;
        this.colorfulballRadius = pBallRadius;
    };
    var BallContent = function(){
        this.toggle = function(){control1 = !control1;};
        this.tornado = false;
        this.wind = false;
        this.windVelocity = windVel[0];
        this.boid = false;
        this.boidMove = false;
    };
    var FireContent = function(){
        this.toggle = function(){control4 = !control4;};
        this.fireIntensity = 50;
    };
    var SnakeContent = function(){
        this.toggle = function(){control2 = !control2;};
        this.number = partBox2.pSys.partCount;
    };

    var text = new GUIContent();
    var balltext = new BallContent();
    var fireText = new FireContent();
    var NetText = new SnakeContent();
    var gui = new dat.GUI();


    gui.add(text, 'hasWalls').onChange(function(val){
        hasWall = val;
        partBox1.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
        partBox2.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
        partBox4.pSys.removeAddWall([WTYPE_XWALL_LO, WTYPE_XWALL_HI, WTYPE_YWALL_LO, WTYPE_YWALL_HI,  WTYPE_ZWALL_LO, WTYPE_ZWALL_HI]);
    });
    gui.add(text, 'wallElasticity').min(0).max(1).onChange(function(val){Kbouncy = val;});
    gui.add(text, 'hascolorfulball').onChange(function(val){
        hasBall = val;
        partBox1.pSys.removeAddWall([WTYPE_PBALL]);
        partBox2.pSys.removeAddWall([WTYPE_PBALL]);
        partBox4.pSys.removeAddWall([WTYPE_PBALL]);
    });
    gui.add(text, 'colorfulballRadius').onChange(function(val){pBallRadius = val;});

    var ball = gui.addFolder('Ball');
    ball.add(balltext, 'toggle');
    ball.add(balltext, 'boid').onChange(function(){partBox1.pSys.removeAddForce([F_FLOCK,F_GRAV_E,F_DRAG]);});
    ball.add(balltext, 'boidMove').onChange(function (){partBox1.pSys.flockMove = !partBox1.pSys.flockMove;});
    ball.add(balltext, 'tornado').onChange(function(){partBox1.pSys.removeAddForce([F_TORNADO]); partBox1.pSys.removeAddWall([TORNADO_CONSTRAINT])});
    ball.add(balltext, 'wind').onChange(function(){partBox1.pSys.removeAddForce([F_WIND]);});
    ball.add(balltext, 'windVelocity').onChange(function(val){windVel[0] = val;});
    ball.open();

    var fire = gui.addFolder('Fire');
    fire.add(fireText, 'toggle');
    fire.add(fireText, 'fireIntensity').min(20).max(100).onChange(function (val){partBox4.pSys.fire_lifetime = val});
    fire.open();


    var springNet = gui.addFolder('springNet');
    springNet.add(NetText, 'toggle');springNet.open();

    gui.add(text, 'solver', ['Euler', 'Implicit-Euler', 'Explicit-Midpoint', 'Adams-Bashforth']).onChange(function(val){
        if (val == 'Euler') solverType = SOLV_EULER;
        else if (val == 'Implicit-Euler') solverType = SOLV_IM_EULER;
        else if (val == 'Explicit-Midpoint') solverType = SOLV_EX_MIDPOINT;
        else if (val == 'Adams-Bashforth') solverType = SOLV_ADAMS_BASHFORTH;
    }).listen();


    gl.clearColor(0.3, 0.3, 0.3, 1);
    gl.enable(gl.DEPTH_TEST); 
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    gl.enable( gl.BLEND );
    gl.blendFunc( gl.SRC_COLOR, gl.ONE_MINUS_SRC_ALPHA );

    vpMatrix = new Matrix4();

    var tick = function() {
        timerAll();
        g_timeStep = animate();
        drawResize();
        requestAnimationFrame(tick, g_canvas);
    };
    tick();
}


function animate() {
    var now = Date.now();	
    var elapsed = now - g_last;	
    g_last = now;
    g_stepCount = (g_stepCount +1)%1000;
    if (elapsed < g_timeStepMin) g_timeStepMin = elapsed;
    else if (elapsed > g_timeStepMax) g_timeStepMax = elapsed;
    return elapsed;
}


function drawResize(){
    var nuCanvas = document.getElementById('webgl');	// get current canvas
    var nuGl = getWebGLContext(nuCanvas);
    
    nuCanvas.width = innerWidth - 16;
    nuCanvas.height = innerHeight*3/4 - 16;

    drawAll(nuGl);
}


function drawAll() {
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    gl.viewport(0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight);
    setCamera();


    if (control0){
        worldBox.switchToMe();
        worldBox.adjust();
        worldBox.draw();   
    }
    if (control1){  // bouncy ball
        partBox1.switchToMe();
        partBox1.adjust();
        partBox1.draw();  
    }
    if (control2){  // spring snake
        partBox2.switchToMe();
        partBox2.adjust();
        partBox2.draw();     
    }
    if (control4){  // fire
        partBox4.switchToMe();
        partBox4.adjust();
        partBox4.draw();     
    }
}


//=================Make objects=====================
function makeGroundGrid() {
	var xcount = 100;
	var ycount = 100;
	var xymax	= 50.0;
	var xColr = new Float32Array([0.8, 0.5, 0.8, 0.5]);
	var yColr = new Float32Array([0.8, 0.5, 0.8, 0.5]);

	gndVerts = new Float32Array(floatsPerVertex*2*(xcount+ycount));
						
	var xgap = xymax/(xcount-1);
	var ygap = xymax/(ycount-1);

	for(v=0, j=0; v<2*xcount; v++, j+= floatsPerVertex) {
		if(v%2==0) {
			gndVerts[j  ] = -xymax + (v  )*xgap;
			gndVerts[j+1] = -xymax;	
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		else {
			gndVerts[j  ] = -xymax + (v-1)*xgap;
			gndVerts[j+1] = xymax;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		gndVerts[j+4] = xColr[0];
		gndVerts[j+5] = xColr[1];
        gndVerts[j+6] = xColr[2];
		gndVerts[j+7] = xColr[3];
	}

	for(v=0; v<2*ycount; v++, j+= floatsPerVertex) {
		if(v%2==0) {
			gndVerts[j  ] = -xymax;
			gndVerts[j+1] = -xymax + (v  )*ygap;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		else {
			gndVerts[j  ] = xymax;
			gndVerts[j+1] = -xymax + (v-1)*ygap;
			gndVerts[j+2] = 0.0;
			gndVerts[j+3] = 1.0;
		}
		gndVerts[j+4] = yColr[0];
		gndVerts[j+5] = yColr[1];
        gndVerts[j+6] = yColr[2];
		gndVerts[j+7] = yColr[3];
	}
}


function makeAxis(){
    axisVerts = new Float32Array([
        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3, 1.0,	// X axis line (origin: gray)
        3.3,  0.0,  0.0, 1.0,		1.0,  0.3,  0.3, 1.0,	// 						 (endpoint: red)
		 
        0.0,  0.0,  0.0, 1.0,       0.3,  0.3,  0.3, 1.0,	// Y axis line (origin: white)
        0.0,  3.3,  0.0, 1.0,		0.3,  1.0,  0.3, 1.0,	//						 (endpoint: green)

        0.0,  0.0,  0.0, 1.0,		0.3,  0.3,  0.3, 1.0,	// Z axis line (origin:white)
        0.0,  0.0,  3.3, 1.0,		0.3,  0.3,  1.0, 1.0	//						 (endpoint: blue)
    ]);
}

function makeCube(){
    cubeVerts = new Float32Array([
        0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1,
        0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1,
        1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1,
        1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1
    ]);
}


function makeSphere() {
    var slices =12;		
    var sliceVerts	= 21;

    var topColr = new Float32Array([0.0, 0.9, 0.0, 1.0]);
    var botColr = new Float32Array([0.0, 0.7, 0.0, 1.0]);
    var errColr = new Float32Array([0.0, 0.5, 0.0, 1.0]);
    var sliceAngle = Math.PI/slices;	

    sphVerts = new Float32Array(((slices*2*sliceVerts)-2) * floatsPerVertex);
                                
    var cosBot = 0.0;				
    var sinBot = 0.0;				
    var cosTop = 0.0;			
    var sinTop = 0.0;
    var j = 0;					
    var isFirstSlice = 1;		
    var isLastSlice = 0;		
    for(s=0; s<slices; s++) {	
        if(s==0) {
            isFirstSlice = 1;		
            cosBot =  0.0; 		
            sinBot = -1.0;		
        }
        else {					
            isFirstSlice = 0;	
            cosBot = cosTop;
            sinBot = sinTop;
        }						
        cosTop = Math.cos((-Math.PI/2) +(s+1)*sliceAngle); 
        sinTop = Math.sin((-Math.PI/2) +(s+1)*sliceAngle);
        if(s==slices-1) isLastSlice=1;
        for(v=isFirstSlice;    v< 2*sliceVerts-isLastSlice;   v++,j+=floatsPerVertex)
        {					
            if(v%2 ==0) { 
                sphVerts[j  ] = cosBot * Math.cos(Math.PI * v/sliceVerts);	
                sphVerts[j+1] = cosBot * Math.sin(Math.PI * v/sliceVerts);	
                sphVerts[j+2] = sinBot;																			// z
                sphVerts[j+3] = 1.0;																				// w.				
            }
            else {	
                sphVerts[j  ] = cosTop * Math.cos(Math.PI * (v-1)/sliceVerts); 
                sphVerts[j+1] = cosTop * Math.sin(Math.PI * (v-1)/sliceVerts);
                sphVerts[j+2] = sinTop;		
                sphVerts[j+3] = 1.0;	
            }
            sphVerts[j+4]=Math.random();
            sphVerts[j+5]=Math.random();
            sphVerts[j+6]=Math.random();
            sphVerts[j+7]=1.0;
        }
    }
}


function drawGroundGrid(modelMatrix, u_ModelMatrix){  
    modelMatrix.scale( 0.1, 0.1, 0.1);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.LINES, gndStart/7, gndVerts.length/7);
}


function drawAxis(modelMatrix, u_ModelMatrix){
	modelMatrix.scale(0.1, 0.1, 0.1);
    gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
    gl.drawArrays(gl.LINES, axisStart/floatsPerVertex, axisVerts.length/floatsPerVertex);
}


function drawCube(modelMatrix, u_ModelMatrix){
    if (hasWall){

        modelMatrix.rotate(90, 1.0, 0.0, 0.0); // rotate ball axis

        // back
        pushMatrix(modelMatrix);
        modelMatrix.scale(2.0, 2.0, 1.0);
        modelMatrix.translate(-0.5, 0.0, 1.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.LINE_LOOP, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // right
        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(1.0, 2.0, 2.0);
        modelMatrix.translate(-1.0, 0.0, 0.5);
        modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.LINE_LOOP, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // left
        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(1.0, 2.0, 2.0);
        modelMatrix.translate(1.0, 0.0, 0.5);
        modelMatrix.rotate(90.0, 0.0, 1.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.LINE_LOOP, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        // up
        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(2.0,2.0, 2.0);
        modelMatrix.translate(-0.5, 1.0, -0.5);
        modelMatrix.rotate(90.0, 1.0, 0.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.LINE_LOOP, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        modelMatrix = popMatrix();
        pushMatrix(modelMatrix);
        modelMatrix.scale(2.0,2.0, 2.0);
        modelMatrix.translate(-0.5, 1.0, -0.5);
        modelMatrix.rotate(90.0, 1.0, 0.0, 0.0);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.LINE_LOOP, cubeStart/floatsPerVertex, cubeVerts.length/floatsPerVertex);

        modelMatrix = popMatrix();
    }
}

function drawSphere(modelMatrix, u_ModelMatrix){
    if (hasBall){
        modelMatrix.translate(pBallCenter[0], pBallCenter[1], pBallCenter[2]);
        modelMatrix.scale(pBallRadius, pBallRadius, pBallRadius);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.TRIANGLE_STRIP, sphStart/floatsPerVertex, sphVerts.length/floatsPerVertex);
    }
}


//====================Control========================
function myKeyDown(ev) {
    switch(ev.code){
		case "ArrowLeft":
            pBallCenter[0] += 0.01;
            break;

        case "ArrowRight":
            pBallCenter[0] -= 0.01;
            break;

        case "ArrowUp":
            pBallCenter[2] += 0.01;
            break;

        case "ArrowDown":
            pBallCenter[2] -= 0.01;
            break;

        case "KeyR":
            // boost velocity only.
            var pSys = partBox1.pSys;
            for (var i = 0; i < pSys.partCount; i++){
                if (pSys.S0[i * PART_MAXVAR + PART_XVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_XVEL] += (Math.random()+1) * 2;
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_XVEL] -= (Math.random()+1) * 2;
                }
                if (pSys.S0[i * PART_MAXVAR + PART_YVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_YVEL] += (Math.random()+1) * 2;  // Also g_drag should be applied??
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_YVEL] -= (Math.random()+1) * 2;
                }
                if (pSys.S0[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] += (Math.random()+1) * 5;
                }else{
                    pSys.S0[i * PART_MAXVAR + PART_ZVEL] -= (Math.random()+1) * 5;
                }
            }
            break;
        
        case "KeyP":
            if (runMode == 3) runMode = 1;
            else runMode = 3;
            break;

        case "Space":
            runMode = 2;
            break;

        default:
            break;
	}
}

var cameraPos = new Vector3([0,5,0]);
var cameraFront = new Vector3([0,-1,0]);
var cameraUp = new Vector3([0,0,1]);
var worldUp = new Vector3([0,0,1]);
var cameraRight = new Vector3([1,0,0]);
var cameraSpeedForward = -0.1;
var cameraSpeedHorizontal = -0.1;
var g_xMclik = 0.0;
var g_yMclik = 0.0;
var forward = 0;
var horizontal = 0;
var mouseSensivity = 0.1;
var yaw = -90;
var pitch = 0;
var viewMatrix = new Matrix4();
var projMatrix = new Matrix4();
var modelMatrix = new Matrix4();
var mvpMat = new Matrix4();

function setCamera() {
    modelMatrix.setIdentity();
    viewMatrix.setIdentity();
    projMatrix.setIdentity();
    viewMatrix.perspective(30.0,   // FOVY: top-to-bottom vertical image angle, in degrees
        g_canvas.width / g_canvas.height,   // Image Aspect Ratio: camera lens width/height
        1.0,   // camera z-near distance (always positive; frustum begins at z = -znear)
        200.0);  // camera z-far distance (always positive; frustum ends at z = -zfar)

    projMatrix.lookAt(	cameraPos.elements[0], cameraPos.elements[1], cameraPos.elements[2], 				// 'Center' or 'Eye Point',
        cameraPos.elements[0] + cameraFront.elements[0], cameraPos.elements[1] + cameraFront.elements[1], cameraPos.elements[2] + cameraFront.elements[2],// look-At point,
        cameraUp.elements[0], cameraUp.elements[1], cameraUp.elements[2]);

    //console.log(projMatrix);
    mvpMat.set(modelMatrix).multiply(viewMatrix).multiply(projMatrix);
}

function updateCameraVectors(){
    const x = -Math.cos(degree_to_radians(yaw)) * Math.cos(degree_to_radians(pitch));
    const z = Math.sin(degree_to_radians(pitch));
    const y = Math.sin(degree_to_radians(yaw)) * Math.cos(degree_to_radians(pitch));
    cameraFront = new Vector3([x,y,z]);
    cameraFront.normalize();
    cameraRight = worldUp.cross(cameraFront);
    cameraUp = cameraFront.cross(cameraRight);
}

function processMouseMovement(xoffset,yoffset){
    xoffset *= mouseSensivity;
    yoffset *= mouseSensivity;

    yaw += xoffset;
    pitch += yoffset;
    if(pitch > 89){
        pitch = 89;
    }
    if(pitch < -89){
        pitch = -89
    }
    updateCameraVectors();
}

function degree_to_radians(degree){
    return degree * (Math.PI/180);
}

var isControlling = true;

document.addEventListener('mousemove',function (ev){
    if(isControlling){
        var rect = ev.target.getBoundingClientRect();	// get canvas corners in pixels
        var xp = ev.clientX - rect.left;									// x==0 at canvas left edge
        var yp = g_canvas.height - (ev.clientY - rect.top);
        var x = (xp - g_canvas.width/2)  / 		// move origin to center of canvas and
            (g_canvas.width/2);		// normalize canvas to -1 <= x < +1,
        var y = (yp - g_canvas.height/2) /		//										-1 <= y < +1.
            (g_canvas.height/2);


        processMouseMovement((x - g_xMclik) * 1000, (y-g_yMclik) * 1000);

        g_xMclik = x;											// Make next drag-measurement from here.
        g_yMclik = y;
    }
});

document.addEventListener("keypress",function (ev){
    if(ev.keyCode == 74 || ev.keyCode == 106){
        if(isControlling){
            isControlling = false;
        }
        else{
            isControlling = true;
        }
    }
})

function timerAll(){
    var movementForward = new Vector3();
    movementForward.elements[0] = cameraFront.elements[0];
    movementForward.elements[1] = cameraFront.elements[1];
    movementForward.elements[2] = cameraFront.elements[2];
    var movementHorizontal = new Vector3();
    movementHorizontal.elements[0] = cameraRight.elements[0];
    movementHorizontal.elements[1] = cameraRight.elements[1];
    movementHorizontal.elements[2] = cameraRight.elements[2];
    movementForward.elements[0] *= (cameraSpeedForward * forward);
    movementForward.elements[1] *= (cameraSpeedForward * forward);
    movementForward.elements[2] *= (cameraSpeedForward * forward);
    movementHorizontal.elements[0] *= (cameraSpeedHorizontal * horizontal);
    movementHorizontal.elements[1] *= (cameraSpeedHorizontal * horizontal);
    movementHorizontal.elements[2] *= (cameraSpeedHorizontal * horizontal);
    cameraPos.elements[0] += (movementForward.elements[0] + movementHorizontal.elements[0]);
    cameraPos.elements[1] += (movementForward.elements[1] + movementHorizontal.elements[1]);
    cameraPos.elements[2] += (movementForward.elements[2] + movementHorizontal.elements[2]);

}

document.addEventListener("keydown",function (ev){

    if(ev.keyCode == 65){
        horizontal = -0.1;
        console.log('h = -0.1');
    }//A
    if(ev.keyCode == 68){
        horizontal = 0.1;
        console.log('h = -0.1');
    }//D
    if(ev.keyCode == 87){
        forward = -0.1;
        console.log('h = -0.1');
    }
    if(ev.keyCode == 83){
        forward = 0.1;
        console.log('h = -0.1');
    }
});

document.addEventListener("keyup",function (ev){
    if(ev.keyCode == 65 || ev.keyCode == 68){
        horizontal = 0;
    }
    if(ev.keyCode == 87 || ev.keyCode == 83){
        forward = 0;
    }
});
