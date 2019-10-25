//
// Created by mrlukasbos on 29-3-19.
//

#include <control/controllers/PidController.h>
#include "utilities/Constants.h"

//**********************************
//Constructor functions
//**********************************
PidController::PidController(double p, double i, double d, double f){
    maxIOutput=0;
    maxError=0;
    errorSum=0;
    maxOutput=0;
    minOutput=0;
    lastActual=0;
    firstRun=true;
    reversed=false;
    outputRampRate=0;
    lastOutput=0;
    outputFilter=0;
    setpointRange=0;

    proportionalGain=p;
    integrativeGain=i;
    derivativeGain=d;
    feedforwardGain=f;
}

PidController::PidController(std::tuple<double, double, double> pid)
        : PidController(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid)) {}

PidController::PidController(std::tuple<double, double, double, double> pidf)
        : PidController(std::get<0>(pidf), std::get<1>(pidf), std::get<2>(pidf), std::get<3>(pidf)) {}


//**********************************
//Configuration functions
//**********************************
void PidController::setPID(double p, double i, double d, double f){
    proportionalGain=p;integrativeGain=i;derivativeGain=d;feedforwardGain=f;
    checkSigns();
}

void PidController::setMaxIOutput(double maximum){
    /* Internally maxError and Izone are similar, but scaled for different purposes. 
     * The maxError is generated for simplifying math, since calculations against 
     * the max error are far more common than changing the I term or Izone. 
     */
    maxIOutput=maximum;
    if(integrativeGain != 0){
        maxError= maxIOutput / integrativeGain;
    }
}

void PidController::setOutputLimits(double output){ setOutputLimits(-output, output);}

void PidController::setOutputLimits(double minimum, double maximum){
    if(maximum<minimum)return;
    maxOutput=maximum;
    minOutput=minimum;

    // Ensure the bounds of the I term are within the bounds of the allowable output swing
    if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
        setMaxIOutput(maximum-minimum);
    }
}

void PidController::setDirection(bool reversed){
    this->reversed=reversed;
}

//**********************************
//Primary operating functions
//**********************************
double PidController::computeOutput(double setpoint, double actual){
    double output;
    double Poutput;
    double Ioutput;
    double Doutput;
    double Foutput;

    //Ramp the setpoint used for calculations if user has opted to do so
    if(setpointRange!=0){
        setpoint=clamp(setpoint,actual-setpointRange,actual+setpointRange);
    }

    //Do the simple parts of the calculations
    double error=setpoint-actual;

    //Calculate F output. Notice, this->depends only on the setpoint, and not the error. 
    Foutput= feedforwardGain * setpoint;

    //Calculate P term
    Poutput= proportionalGain * error;

    //If this->is our first time running this-> we don't actually _have_ a previous input or output. 
    //For sensor, sanely assume it was exactly where it is now.
    //For last output, we can assume it's the current time-independent outputs. 
    if(firstRun){
        lastActual=actual;
        lastOutput=Poutput+Foutput;
        firstRun=false;
    }


    //Calculate D Term
    //Note, this->is negative. this->actually "slows" the system if it's doing
    //the correct thing, and small values helps prevent output spikes and overshoot 

    Doutput= -derivativeGain * (actual - lastActual) * rtt::ai::Constants::TICK_RATE();
    lastActual=actual;



    //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
    // 1. maxIoutput restricts the amount of output contributed by the Iterm.
    // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput	
    Ioutput= integrativeGain * errorSum / rtt::ai::Constants::TICK_RATE();
    if(maxIOutput!=0){
        Ioutput=clamp(Ioutput,-maxIOutput,maxIOutput);
    }

    //And, finally, we can just add the terms up
    output=Foutput + Poutput + Ioutput + Doutput;

    //Figure out what we're doing with the error.
    if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
        errorSum=error;
        // reset the error sum to a sane level
        // Setting to current error ensures a smooth transition when the P term 
        // decreases enough for the I term to start acting upon the controller
        // From that point the I term will build up as would be expected
    }
    else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
        errorSum=error;
    }
    else if(maxIOutput!=0){
        errorSum=clamp(errorSum+error,-maxError,maxError);
        // In addition to output limiting directly, we also want to prevent I term 
        // buildup, so restrict the error directly
    }
    else{
        errorSum+=error;
    }

    //Restrict output to our specified output and ramp limits
    if(outputRampRate!=0){
        output=clamp(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
    }
    if(minOutput!=maxOutput){
        output=clamp(output, minOutput,maxOutput);
    }
    if(outputFilter!=0){
        output=lastOutput*outputFilter+output*(1-outputFilter);
    }

    lastOutput=output;
    return output;
}

void PidController::reset(){
    firstRun=true;
    errorSum=0;
}

void PidController::setOutputRampRate(double rate){
    outputRampRate=rate;
}

void PidController::setSetpointRange(double range){
    setpointRange=range;
}

void PidController::setOutputFilter(double strength){
    if(strength==0 || bounded(strength,0,1)){
        outputFilter=strength;
    }
}

//**************************************
// Helper functions
//**************************************
double PidController::clamp(double value, double min, double max){
    if(value > max){ return max;}
    if(value < min){ return min;}
    return value;
}

bool PidController::bounded(double value, double min, double max){
    return (min<value) && (value<max);
}

void PidController::checkSigns(){
    if(reversed){	//all values should be below zero
        if(proportionalGain > 0) proportionalGain*=-1;
        if(integrativeGain > 0) integrativeGain*=-1;
        if(derivativeGain > 0) derivativeGain*=-1;
        if(feedforwardGain > 0) feedforwardGain*=-1;
    }
    else{	//all values should be above zero
        if(proportionalGain < 0) proportionalGain*=-1;
        if(integrativeGain < 0) integrativeGain*=-1;
        if(derivativeGain < 0) derivativeGain*=-1;
        if(feedforwardGain < 0) feedforwardGain*=-1;
    }
}

void PidController::setPID(std::tuple<double, double, double> pid, double f) {
    this->setPID(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid), f);
}
