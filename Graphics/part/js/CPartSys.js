SOLV_EULER = 0;
SOLV_IM_EULER = 1;
SOLV_EX_MIDPOINT = 2;
SOLV_ITER_IM_EULER = 3;
SOLV_ITER_IM_MIDPOINT = 4;
SOLV_ADAMS_BASHFORTH = 5;
SOLV_MAX = 6;
solverType = SOLV_EX_MIDPOINT;

PTYPE_DEAD = 0;   // DEAD PARTICLE
PTYPE_ALIVE = 1;  // 'default' particle
PTYPE_BALL = 2;   // small bouncy round shiny sphere particle
PTYPE_MAXVAR = 3;

PART_MASS = 0;
PART_XPOS = 1;
PART_YPOS = 2;
PART_ZPOS = 3;
PART_WPOS = 4;
PART_XVEL = 5;
PART_YVEL = 6;
PART_ZVEL = 7;
PART_X_FTOT = 8;
PART_Y_FTOT = 9;
PART_Z_FTOT = 10;
PART_SIZE = 11;
PART_MAXAGE = 12;
PART_AGE = 13;  // # of frame-times since creation/initialization
PART_R = 14;
PART_G = 15;
PART_B = 16;
PART_A = 17;
PART_MAXVAR = 18;


isFountain = false;
isFixed = true;

springLen = 0.3;
springStiffness = 4;
springDamp = 0.1;
windVel = [1.0, 0.0, 0.0];
T_center = [0, 0, 0];
sprMass = 0.5;

cnt = 0;

class CPartSys {
    //-------State Vectors-----------------------!
    partCount = 0;    // # of particles held in each state vector. (if <=0, state vectors set to NULL)
    S0; S0dot;  // state vector holding CURRENT state of particles, and its time derivative s0dot.
    S1; S1dot;  // NEXT state, and its time-derivative.
    SM; SMdot;  // midpoint state vector and its time-deriv.
    
    //-------Force-List Vector(s)----------------
    forcerCount = 0;  // # of forcer-making objects held in the
                  // dyn. alloc'd list at pF0 (if needed, in
                  // pF1, pFM, pF0dot,pF1dot,pFMdot as well).
    F0;          // f0; forcer-vector-- dyn. alloc'd list of all CURRENT force-making objects,

    //-------Wall-Vectors(s)---------------
    wallCount = 0;    // # of constraint-making objects (CWall obj)
                  // held in the dyn. alloc'd list at pC0.
    C0;          // c0; constraint-vector--dyn. alloc'd list
                  // of all CURRENT constraint-making objects

    INIT_VEL =  0.05 * 20.0;

    springNETrow = 0;
    springNetcol = 0;

    flockMove = false;
    fire_lifetime = 50;

    initBouncyBall(partCount, forces, walls){
        this.partCount = partCount;
        this.forcerCount = forces.length;
        this.wallCount = walls.length;
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S0dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);
        for (var i = 0, j = 0; i < this.partCount; i++, j += PART_MAXVAR){  // init mass
            this.S0[j + PART_MASS] = 1 ;
            this.S0[j + PART_XPOS] = 1.5 * (Math.random()-0.5);
            this.S0[j + PART_YPOS] = 0;
            this.S0[j + PART_ZPOS] = 0;
            this.S0[j + PART_WPOS] = 1.0;
            this.S0[j + PART_XVEL] = Math.random();
            this.S0[j + PART_YVEL] = 0;
            this.S0[j + PART_ZVEL] = Math.random();
            this.S0[j + PART_X_FTOT] = 0.0;
            this.S0[j + PART_Y_FTOT] = 0.0;
            this.S0[j + PART_Z_FTOT] = 0.0;
            this.S0[j + PART_SIZE] = 8.0;
            this.S0[j + PART_MAXAGE] = 100 + 100*Math.random();//
            this.S0[j + PART_AGE] = this.S0[j + PART_MAXAGE];
            this.S0[j + PART_R] = 0.3;
            this.S0[j + PART_G] = 0.3;
            this.S0[j + PART_B] = 0.3;
            this.S0[j + PART_A] = 0.9;
        }
 
        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.S1dot.fill(0);
        this.SMdot.fill(0);
        
        this.solvType = solverType;
        // initialize f0, the list of CForcer for s0
        this.F0 = []; // Do we need single CPart/force object?
        for (var i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));
        }

        // initialize constraints c0 used to adjust s1
        this.C0 = [];
        for (var i = 0; i < this.wallCount; i++){
            this.C0.push(new CWall(walls[i]));
        }
    }


    initSpringNet(row,col, forces, walls){
        this.partCount = row * col;  // Spring vertex #
        this.forcerCount = forces.length;
        this.wallCount = walls.length;

        this.springNetcol = col;
        this.springNETrow = row;

        // initialize s1 for all particles
        this.S0 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S0dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1 = new Float32Array(this.partCount * PART_MAXVAR);
        this.S1dot = new Float32Array(this.partCount * PART_MAXVAR);
        this.SM = new Float32Array(this.partCount * PART_MAXVAR);
        this.SMdot = new Float32Array(this.partCount * PART_MAXVAR);

        for(var i = 0, k = 0; i < row; i++){
            for(var j = 0 ; j < col ; j++, k += PART_MAXVAR){
                this.S0[k + PART_MASS] = 0.05;
                this.S0[k + PART_XPOS] = j * 0.1;
                this.S0[k + PART_YPOS] = 0;
                this.S0[k + PART_ZPOS] = 1.5 - i * 0.1;
                this.S0[k + PART_WPOS] =  1.0;
                this.S0[k + PART_XVEL] =  0.0;
                this.S0[k + PART_YVEL] =  0.0;
                this.S0[k + PART_ZVEL] =  0.0;
                this.S0[k + PART_X_FTOT] =  0.0;
                this.S0[k + PART_Y_FTOT] =  0.0;
                this.S0[k + PART_Z_FTOT] =  0.0;
                this.S0[k + PART_SIZE] = 20.0;
                this.S0[k + PART_AGE] =  0.0;
                this.S0[k + PART_R] =  1.0;
                this.S0[k + PART_G] =  1.0;
                this.S0[k + PART_B] =  1.0;
                this.S0[k + PART_A] =  1.0;
            }
        }


        // initialize s1 as a copy from s0
        this.S1.set(this.S0);
        this.SM.set(this.S0);

        // initialize s0dot as a copy from s0 but set all to 0
        this.S0dot.fill(0);
        this.S1dot.fill(0);
        this.SMdot.fill(0);

        this.solvType = solverType;

        this.F0 = [];
        for (i = 0; i < this.forcerCount; i++){
            this.F0.push(new CForcer(forces[i]));

        }

        for(var i = 0 ; i < this.F0.length ; i++){

            if(this.F0[i].forceType == F_SPRINGNET){
                for(var j = 0 ; j < row; j++){
                    for(var k = 0 ; k < col ; k++){
                        if(j == row - 1 && k == col - 1){
                            continue;
                        }
                        else if(j == row - 1){
                            this.F0[i].springNetE1.push(j * col + k);
                            this.F0[i].springNetE2.push(j * col + k + 1);
                        }
                        else if(k == col - 1){
                            this.F0[i].springNetE1.push(j * col + k);
                            this.F0[i].springNetE2.push((j + 1)*col + k);
                        }
                        else{
                            this.F0[i].springNetE1.push(j * col + k);
                            this.F0[i].springNetE2.push(j * col + k + 1);
                            this.F0[i].springNetE1.push(j * col + k);
                            this.F0[i].springNetE2.push((j + 1)*col + k);
                        }
                    }
                }
            }
        }

        this.C0 = [];
        for (i = 0; i < this.wallCount; i++){
            this.C0.push(new CWall(walls[i]));
        }
    }

    applyAllForces(S, F){
        // Clear force accumulators for each particle
        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
            S[k + PART_X_FTOT] = 0.0;
            S[k + PART_Y_FTOT] = 0.0;
            S[k + PART_Z_FTOT] = 0.0;            
        }
        
        // Step through the forcers. Accumulate all forces.
        for (var j = 0; j < this.forcerCount; j++){
            switch(F[j].forceType){
                case -F_GRAV_E:  // disabled gravity. Do nothing.
                    break;
                    
                case F_GRAV_E:  // Earth gravity.
                    for (var i = 0; i < this.partCount; i++){  // For every particle
                        // magnitude of gravity's force. F=ma
                        var mag = S[i * PART_MAXVAR + PART_MASS] * F[j].gravConst;  
                        // make a vector: scale our unit vector in the 'down' direction:
                        S[i * PART_MAXVAR + PART_X_FTOT] += mag * F[j].downDir[0];
                        S[i * PART_MAXVAR + PART_Y_FTOT] += mag * F[j].downDir[1];
                        S[i * PART_MAXVAR + PART_Z_FTOT] += mag * F[j].downDir[2];
                    }
                    break;

                case F_DRAG:  // viscous drag: force = -velocity*K_drag.
                    for(var i = 0; i < this.partCount; i++){
                        // add to force-accumulator
                        S[i * PART_MAXVAR + PART_X_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_XVEL];
                        S[i * PART_MAXVAR + PART_Y_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_YVEL];
                        S[i * PART_MAXVAR + PART_Z_FTOT] += -F[j].K_drag * S[i * PART_MAXVAR + PART_ZVEL];
                    }
                    break;
                
                case F_WIND:
                        F[j].D_wind = 0.1;
                        F[j].v_wind = windVel;
    
                        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                            S[k + PART_X_FTOT] += F[j].D_wind * F[j].v_wind[0];
                            S[k + PART_Y_FTOT] += F[j].D_wind * F[j].v_wind[1];
                            S[k + PART_Z_FTOT] += F[j].D_wind * F[j].v_wind[2];
                        }
                    break;
                
                case F_TORNADO:
                        for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                            var dis = distance2D(T_center.slice(0,2), 
                                                S.slice(k + PART_XPOS, k + PART_ZPOS));

                            //the upward force
                            S[k + PART_Z_FTOT] += 1/dis;

                            //the inward force
                            S[k + PART_X_FTOT] += (T_center[0] - S[k + PART_XPOS])/(dis*dis);
                            S[k + PART_Y_FTOT] += (T_center[1] - S[k + PART_YPOS])/(dis*dis);

                            //reduce the inward force because of the height
                            if (S[k + PART_X_FTOT] > 0){S[k + PART_X_FTOT] -= Math.pow(S[k + PART_ZPOS] - T_center[2], 2)/dis;}
                            else S[k + PART_X_FTOT] += Math.pow(S[k + PART_ZPOS] - T_center[2], 2)/dis;
                            if (S[k + PART_Y_FTOT] > 0){S[k + PART_Y_FTOT] -= Math.pow(S[k + PART_ZPOS] - T_center[2], 2)/dis;}
                            else S[k + PART_Y_FTOT] += Math.pow(S[k + PART_ZPOS] - T_center[2], 2)/dis;

                            //rotation force
                            S[k + PART_X_FTOT] += (-S[k + PART_YPOS] + T_center[1])/dis*dis;
                            S[k + PART_Y_FTOT] += (S[k + PART_XPOS] - T_center[0])/dis*dis;
                        }
                    break;

                case F_FLOCK:
                    var flockcenter = F[j].FlockCenter;

                    var kc1 = 2.5;
                    var kc2 = 1.25;
                    var k_dist = 0.5;
                    for(var i = 0 ; i < this.partCount ; i++){
                        var currPos = S.slice(i * PART_MAXVAR + PART_XPOS, i * PART_MAXVAR + PART_ZPOS + 1);
                        var dis = distance3D(flockcenter,currPos);
                        var direction = new Vector3([flockcenter[0] - currPos[0],flockcenter[1] - currPos[1], flockcenter[2] - currPos[2]]);
                        direction.normalize();
                        var force = 0;
                        if(dis > k_dist){
                            force = (dis - k_dist) * kc1;
                            S[i*PART_MAXVAR + PART_X_FTOT] += direction.elements[0] * force;
                            S[i*PART_MAXVAR + PART_Y_FTOT] += direction.elements[1] * force;
                            S[i*PART_MAXVAR + PART_Z_FTOT] += direction.elements[2] * force;
                        }
                        else if(dis < k_dist){
                            force = (dis - k_dist) * kc2;
                            S[i*PART_MAXVAR + PART_X_FTOT] += direction.elements[0] * force;
                            S[i*PART_MAXVAR + PART_Y_FTOT] += direction.elements[1] * force;
                            S[i*PART_MAXVAR + PART_Z_FTOT] += direction.elements[2] * force;
                        }

                    }
                    if(this.flockMove){
                        F[j].FlockCenter[0] += 0.1 * (Math.random() - 0.5);
                        F[j].FlockCenter[1] += 0.1 * (Math.random() - 0.5);
                        F[j].FlockCenter[2] += 0.1 * (Math.random() - 0.5);
                    }

                    break;


                case F_FIRE:
                    for(var k = 0, i = 0 ; i < this.partCount ; k+=PART_MAXVAR, i++){
                        var zpos = S[k + PART_ZPOS];
                        if(zpos >= 0.01){
                            var upforce = F[j].k_up/zpos;
                            S[k + PART_Z_FTOT] += upforce;
                        }
                    }
                    break;

                case F_SPRINGNET:
                    var springNum = F[j].springNetE2.length;

                    console.log(springNum);
                    for(var k = 0 ; k < springNum ; k++){
                        var p1Index = F[j].springNetE1[k];
                        var p2Index = F[j].springNetE2[k];
                        console.log(p1Index+" "+p2Index);
                        var direction = new Vector3([S[p1Index * PART_MAXVAR + PART_XPOS] - S[p2Index * PART_MAXVAR + PART_XPOS],//p1->p2
                            S[p1Index * PART_MAXVAR + PART_YPOS] - S[p2Index * PART_MAXVAR + PART_YPOS],
                            S[p1Index * PART_MAXVAR + PART_ZPOS] - S[p2Index * PART_MAXVAR + PART_ZPOS]]);
                        direction.normalize();
                        var distance = Math.sqrt(Math.pow(S[p1Index * PART_MAXVAR + PART_XPOS] - S[p2Index * PART_MAXVAR + PART_XPOS], 2)+
                            Math.pow(S[p1Index * PART_MAXVAR + PART_YPOS] - S[p2Index * PART_MAXVAR + PART_YPOS], 2)+
                            Math.pow(S[p1Index * PART_MAXVAR + PART_ZPOS] - S[p2Index * PART_MAXVAR + PART_ZPOS], 2));



                        var shapeChange = F[j].K_restLength - distance;
                        var outward = true;
                        if(shapeChange > 0){
                            outward = false;
                        }
                        else{
                            shapeChange = -shapeChange;
                        }
                        var forceShape = shapeChange*F[j].K_spring;

                        var forceTotalScale = forceShape;
                        console.log(forceTotalScale);
                        //for the first particle e1
                        if(outward == true){//direction e1->e2 so force should be on opposite direction of e1->e2
                            var springForcep1 = new Vector3([-direction.elements[0] * forceTotalScale,-direction.elements[1] * forceTotalScale,-direction.elements[2] * forceTotalScale]);
                            S[p1Index*PART_MAXVAR + PART_X_FTOT] += springForcep1.elements[0];
                            S[p1Index*PART_MAXVAR + PART_Y_FTOT] += springForcep1.elements[1];
                            S[p1Index*PART_MAXVAR + PART_Z_FTOT] += springForcep1.elements[2];
                        }
                        else{
                            var springForcep1 = new Vector3([direction.elements[0] * forceTotalScale,direction.elements[1] * forceTotalScale,direction.elements[2] * forceTotalScale]);
                            S[p1Index*PART_MAXVAR + PART_X_FTOT] += springForcep1.elements[0];
                            S[p1Index*PART_MAXVAR + PART_Y_FTOT] += springForcep1.elements[1];
                            S[p1Index*PART_MAXVAR + PART_Z_FTOT] += springForcep1.elements[2];
                        }
                        //for the second particle e2
                        if(outward == true){
                            var springForcep2 = new Vector3([direction.elements[0] * forceTotalScale,direction.elements[1] * forceTotalScale,direction.elements[2] * forceTotalScale]);
                            S[p2Index*PART_MAXVAR + PART_X_FTOT] += springForcep2.elements[0];
                            S[p2Index*PART_MAXVAR + PART_Y_FTOT] += springForcep2.elements[1];
                            S[p2Index*PART_MAXVAR + PART_Z_FTOT] += springForcep2.elements[2];
                        }
                        else{
                            var springForcep2 = new Vector3([-direction.elements[0] * forceTotalScale,-direction.elements[1] * forceTotalScale,-direction.elements[2] * forceTotalScale]);
                            S[p2Index*PART_MAXVAR + PART_X_FTOT] += springForcep2.elements[0];
                            S[p2Index*PART_MAXVAR + PART_Y_FTOT] += springForcep2.elements[1];
                            S[p2Index*PART_MAXVAR + PART_Z_FTOT] += springForcep2.elements[2];
                        }
                    }
                    break;

                case F_NONE:
                    break;

                default:
                    break;
            }
        }
    }

    dotfinder(S0dot, S0){
        for (var i = 0; i < this.partCount; i++){  // For every particle

            S0dot[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XVEL];
            S0dot[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YVEL];
            S0dot[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZVEL];
            S0dot[i * PART_MAXVAR + PART_WPOS] = 0.0;

            // Compute a=F/m as sdot velocity.
            S0dot[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_X_FTOT] / S0[i * PART_MAXVAR + PART_MASS];
            S0dot[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_Y_FTOT] / S0[i * PART_MAXVAR + PART_MASS];
            S0dot[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_Z_FTOT] / S0[i * PART_MAXVAR + PART_MASS];

        }
    }

    solver(timeStep, S0, S0dot, S1){
        this.solvType = solverType;
        var h = timeStep * 0.001;
        switch (this.solvType){

            case SOLV_EULER:
                for (var i = 0; i < this.partCount; i++){
                    S1[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + S0dot[i * PART_MAXVAR + PART_XPOS] * h;
                    S1[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + S0dot[i * PART_MAXVAR + PART_YPOS] * h;
                    S1[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + S0dot[i * PART_MAXVAR + PART_ZPOS] * h;
                    S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + S0dot[i * PART_MAXVAR + PART_XVEL] * h;
                    S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + S0dot[i * PART_MAXVAR + PART_YVEL] * h;
                    S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + S0dot[i * PART_MAXVAR + PART_ZVEL] * h;
                }
                break;
            case SOLV_IM_EULER:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + S0dot[i * PART_MAXVAR + PART_XVEL]* h;
                    S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + S0dot[i * PART_MAXVAR + PART_YVEL]* h;
                    S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + S0dot[i * PART_MAXVAR + PART_ZVEL]* h;
                    S1[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + S1[i * PART_MAXVAR + PART_XVEL] * h;
                    S1[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + S1[i * PART_MAXVAR + PART_YVEL] * h;
                    S1[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + S1[i * PART_MAXVAR + PART_ZVEL] * h;
                }
                break;

            case SOLV_EX_MIDPOINT:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    this.SM.set(S0);
                    this.SMdot.fill(0);
                    this.SM[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + S0dot[i * PART_MAXVAR + PART_XPOS] * h/2;
                    this.SM[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + S0dot[i * PART_MAXVAR + PART_YPOS] * h/2;
                    this.SM[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + S0dot[i * PART_MAXVAR + PART_ZPOS] * h/2;
                    this.SM[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + S0dot[i * PART_MAXVAR + PART_XVEL] * h/2;
                    this.SM[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + S0dot[i * PART_MAXVAR + PART_YVEL] * h/2;
                    this.SM[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + S0dot[i * PART_MAXVAR + PART_ZVEL] * h/2;
                    this.dotfinder(this.SMdot, this.SM);
                    S1[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + this.SMdot[i * PART_MAXVAR + PART_XPOS] * h;
                    S1[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + this.SMdot[i * PART_MAXVAR + PART_YPOS] * h;
                    S1[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + this.SMdot[i * PART_MAXVAR + PART_ZPOS] * h;
                    S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + this.SMdot[i * PART_MAXVAR + PART_XVEL] * h;
                    S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + this.SMdot[i * PART_MAXVAR + PART_YVEL] * h;
                    S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + this.SMdot[i * PART_MAXVAR + PART_ZVEL] * h;
                }
                break;

            case SOLV_ADAMS_BASHFORTH:
                for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                    this.dotfinder(this.S1dot, S1);
                    S1[i * PART_MAXVAR + PART_XPOS] = S0[i * PART_MAXVAR + PART_XPOS] + (S0dot[i * PART_MAXVAR + PART_XPOS]*3/2 - this.S1dot[i * PART_MAXVAR + PART_XPOS]/2) * h;
                    S1[i * PART_MAXVAR + PART_YPOS] = S0[i * PART_MAXVAR + PART_YPOS] + (S0dot[i * PART_MAXVAR + PART_YPOS]*3/2 - this.S1dot[i * PART_MAXVAR + PART_YPOS]/2) * h;
                    S1[i * PART_MAXVAR + PART_ZPOS] = S0[i * PART_MAXVAR + PART_ZPOS] + (S0dot[i * PART_MAXVAR + PART_ZPOS]*3/2 - this.S1dot[i * PART_MAXVAR + PART_ZPOS]/2) * h;
                    S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL] + (S0dot[i * PART_MAXVAR + PART_XVEL]*3/2 - this.S1dot[i * PART_MAXVAR + PART_XVEL]/2) * h;
                    S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL] + (S0dot[i * PART_MAXVAR + PART_YVEL]*3/2 - this.S1dot[i * PART_MAXVAR + PART_YVEL]/2) * h;
                    S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL] + (S0dot[i * PART_MAXVAR + PART_ZVEL]*3/2 - this.S1dot[i * PART_MAXVAR + PART_ZVEL]/2) * h;
                }
                break;

        }
    }


    doConstraints(S1, S0, W){

        var hasFire = typeof(W.find(w => w.wallType == FIRE_CONSTRAINT));
        // step through constraints
        for (var j = 0; j < this.wallCount; j++){
            if (hasFire != "undefined"){  // has fire constraint
                W[j].Kbouncy = 0;
            } else {
                W[j].Kbouncy = Kbouncy;
            }


            switch(W[j].wallType){
                case WTYPE_GROUND:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                            S1[i * PART_MAXVAR + PART_ZVEL] = 0.985*S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_YWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_YPOS] < W[j].ymin && S1[i * PART_MAXVAR + PART_YVEL] < 0.0){  // collision
                            S1[i * PART_MAXVAR + PART_YPOS] = W[j].ymin;
                            S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL]; // Still apply drag??
                            if (S1[i * PART_MAXVAR + PART_YVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_YWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_YPOS] > W[j].ymax && S1[i * PART_MAXVAR + PART_YVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_YPOS] = W[j].ymax;
                            S1[i * PART_MAXVAR + PART_YVEL] = S0[i * PART_MAXVAR + PART_YVEL];
                            if (S1[i * PART_MAXVAR + PART_YVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_YVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_YVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_XWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_XPOS] < W[j].xmin && S1[i * PART_MAXVAR + PART_XVEL] < 0.0){
                            S1[i * PART_MAXVAR + PART_XPOS] = W[j].xmin;
                            S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL];
                            if (S1[i * PART_MAXVAR + PART_XVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_XWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_XPOS] > W[j].xmax && S1[i * PART_MAXVAR + PART_XVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_XPOS] = W[j].xmax;
                            S1[i * PART_MAXVAR + PART_XVEL] = S0[i * PART_MAXVAR + PART_XVEL];
                            if (S1[i * PART_MAXVAR + PART_XVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_XVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_XVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_ZWALL_LO:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] < W[j].zmin && S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){  // To be edited
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmin;
                            S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] < 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_ZWALL_HI:
                    for (var i = 0; i < this.partCount; i++){
                        if (S1[i * PART_MAXVAR + PART_ZPOS] > W[j].zmax && S1[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                            S1[i * PART_MAXVAR + PART_ZPOS] = W[j].zmax;
                            S1[i * PART_MAXVAR + PART_ZVEL] = S0[i * PART_MAXVAR + PART_ZVEL];
                            if (S1[i * PART_MAXVAR + PART_ZVEL] > 0.0){
                                S1[i * PART_MAXVAR + PART_ZVEL] *= -W[j].Kbouncy;
                            } else {
                                S1[i * PART_MAXVAR + PART_ZVEL] *= W[j].Kbouncy;
                            }
                        }
                    }
                    break;

                case WTYPE_PBALL:
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        var dis = distance3D(pBallCenter, S1.slice(k + PART_XPOS, k + PART_ZPOS + 1));
                        if (dis <= pBallRadius + ballRadius){
                            var ballPos = S1.slice(k + PART_XPOS,k+PART_ZPOS + 1);
                            var speedChangeDirection = new Vector3([ballPos[0] - pBallCenter[0],
                                ballPos[1] - pBallCenter[1],
                                ballPos[2] - pBallCenter[2]]);
                            var ballVelocity = S1.slice(k + PART_XVEL, k + PART_ZVEL + 1);
                            ballVelocity = new Vector3([ballVelocity[0],ballVelocity[1],ballVelocity[2]]);
                            speedChangeDirection.normalize();
                            var speedchangeMagnitude = speedChangeDirection.dot(ballVelocity);
                            ballVelocity.elements[0] -= 2*speedchangeMagnitude*speedChangeDirection.elements[0];
                            ballVelocity.elements[1] -= 2*speedchangeMagnitude*speedChangeDirection.elements[1];
                            ballVelocity.elements[2] -= 2*speedchangeMagnitude*speedChangeDirection.elements[2];

                            S1[k + PART_XVEL] = ballVelocity.elements[0];
                            S1[k + PART_YVEL] = ballVelocity.elements[1];
                            S1[k + PART_ZVEL] = ballVelocity.elements[2];
                        }
                    }
                    break;

                case FIRE_CONSTRAINT:
                    console.log(this.fire_lifetime);
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        S1[k + PART_AGE] = S0[k+ PART_AGE] - 1;
                        S1[k + PART_MASS] = S0[k + PART_MASS] - 0.05;

                        if (S1[k + PART_MASS] < 0.05){
                            S1[k + PART_MASS] = 0.05;
                        }

                        var lifeLeftPercent = S1[k + PART_AGE]/S1[k + PART_MAXAGE];


                        S1[k + PART_SIZE] = 7 + lifeLeftPercent*3;

                        if (lifeLeftPercent > 0.95){ // white, a little blue
                            S1[k + PART_R] = 1.0;
                            S1[k + PART_G] = 1.0;
                            S1[k + PART_B] = 0.2;
                        }
                        else if (lifeLeftPercent > 0.5){  //yellow to orange
                            S1[k + PART_B] = 0.0;
                            S1[k + PART_G] = lifeLeftPercent - 0.5;
                        }
                        else {
                            S1[k + PART_G] = 0;
                            S1[k + PART_R] = lifeLeftPercent + 0.2;
                        }

                        if (S1[k + PART_AGE] < 0 ){  // Dead
                            // Restart life cycle.
                            this.roundRand();
                            S1[k + PART_MASS] = 1 + Math.random()*2;
                            S1[k + PART_XPOS] = this.randX;
                            S1[k + PART_YPOS] = this.randY;
                            S1[k + PART_ZPOS] = this.randZ;
                            S1[k + PART_WPOS] = 1.0;
                            S1[k + PART_XVEL] = this.INIT_VEL*(Math.random()*0.6 - 0.3);
                            S1[k + PART_YVEL] = this.INIT_VEL*(Math.random()*0.6 - 0.3);
                            S1[k + PART_ZVEL] = this.INIT_VEL*(0.2 + Math.random());
                            S1[k + PART_SIZE] = 10.0;
                            S1[k + PART_MAXAGE] = this.fire_lifetime + 50 * Math.random();
                            S1[k + PART_AGE] = S1[k + PART_MAXAGE];  // Reconstruction here
                            S1[k + PART_R] = 1.0;
                            S1[k + PART_G] = 1.0;
                            S1[k + PART_B] = 0.2;
                            S1[k + PART_A] = 0.7;
                        }
                    }
                    break;

                case TORNADO_CONSTRAINT:
                    for (var i = 0, k = 0; i < this.partCount; i++, k += PART_MAXVAR){
                        // --Age
                        S1[k + PART_AGE] = S0[k + PART_AGE] - 1;

                        lifeLeftPercent = S1[k + PART_AGE]/S1[k+PART_MAXAGE];
                        if(lifeLeftPercent < 0.6){
                            S1[k + PART_R] = lifeLeftPercent + 0.4;
                            S1[k + PART_G] = lifeLeftPercent + 0.4;
                            S1[k + PART_B] = lifeLeftPercent + 0.4;
                        }


                        if (S1[k + PART_AGE] < 0 ){  // Dead
                            // Restart life cycle.
                            S1[k + PART_MASS] = 0.1;
                            S1[k + PART_XPOS] = 3 * (Math.random()-0.5);
                            S1[k + PART_YPOS] = 3 * (Math.random()-0.5);
                            S1[k + PART_ZPOS] = 0;
                            S1[k + PART_WPOS] = 1.0;
                            S1[k + PART_XVEL] = 0;
                            S1[k + PART_YVEL] = 0;
                            S1[k + PART_ZVEL] = 0;
                            S1[k + PART_X_FTOT] = 0.0;
                            S1[k + PART_Y_FTOT] = 0.0;
                            S1[k + PART_Z_FTOT] = 0.0;  
                            S1[k + PART_AGE] = 80;
                            S1[k + PART_R] = 1.0;
                            S1[k + PART_G] = 1.0;
                            S1[k + PART_B] = 1.0;
                            S1[k + PART_A] = 0.9;
                        }
                    }
                    break;
            
                default:
                    break;
            }
        }
    }


    drawMe(S, modelMatrix, u_ModelMatrix){
        // Draw all particles together
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, S, gl.DYNAMIC_DRAW);
        gl.uniformMatrix4fv(u_ModelMatrix, false, modelMatrix.elements);
        gl.drawArrays(gl.POINTS, 0, this.partCount);
    }


    stateVecSwap(A, B){
        [B, A] = [A, B];
        return [A, B]
    }

    roundRand(){
        do {
            this.randX = 2.0*Math.random() -1.0;
            this.randY = 2.0*Math.random() -1.0;
            this.randZ = 2.0*Math.random() -1.0;
        }  
        while(this.randX*this.randX + 
              this.randY*this.randY + 
              this.randZ*this.randZ >= 1.0
              && this.randZ < 0.0);
        this.randX *= 0.5;
        this.randY *= 0.5;
        this.randZ *= 0.001;
    }

    removeAddForce(fNumbers){
        for (var i = 0; i < fNumbers.length; i++){
            var obj = this.F0.find(f => f.forceType == fNumbers[i]);
            if (typeof(obj) != "undefined"){
                this.F0.splice(this.F0.indexOf(obj), 1);
            } else {
                this.F0.push(new CForcer(fNumbers[i]));
            }
        }
        this.forcerCount = this.F0.length;
    }

    removeAddWall(cNumbers){
        for (var i = 0; i < cNumbers.length; i++){
            var obj = this.C0.find(w => w.wallType == cNumbers[i]);
            if (typeof(obj) != "undefined"){  // remove
                this.C0.splice(this.C0.indexOf(obj), 1);
            } else {  // add
                this.C0.push(new CWall(cNumbers[i]));
            }
        }
        this.wallCount = this.C0.length;
    }
    
}

function distance3D(p1, p2){  // Calculate distance between two 3d points
    var x = Math.abs(p1[0] - p2[0]);
    var y = Math.abs(p1[1] - p2[1]);
    var z = Math.abs(p1[2] - p2[2]);

    return Math.sqrt(x*x + y*y + z*z);
}

function distance2D(p1, p2){  // Calculate distance between two 3d points
    var x = Math.abs(p1[0] - p2[0]);
    var y = Math.abs(p1[1] - p2[1]);

    return Math.sqrt(x*x + y*y);
}

function dot(v1, v2){
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}