// RPI.cpp

#ifdef _WINDOWS
#include "stdafx.h"
#else
#include <stdlib.h>     /* abs */
// ???
#endif

#include "RPI.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// access control constants
/*
#define uStepResolution  pRpi->mcs.uStepResolution
#define uFrontLegStart   pRpi->mcs.uFrontLegStart 
#define uHindLegStart    pRpi->mcs.uHindLegStart  
#define uStepLengthF      pRpi->mcs.uStepLengthF
#define uStepLengthH      pRpi->mcs.uStepLengthH
#define uWalkLevel       pRpi->mcs.uWalkLevel     
#define uPawLift         pRpi->mcs.uPawLift       
#define bSingleStep      pRpi->mcs.bSingleStep
*/
#define uStepResolution  14  // length of kinematic-step   (alles in mm)
#define uFrontLegStart   -10  // x start pos. of pace
#define uHindLegStart    -20
#define uStepLengthF     70  // length of one leg move on the ground
#define uStepLengthH     70  // length of one leg move on the ground
#define uWalkLevel       10  // y walking level of init and walking
#define uPawLift         10  // H?he ?ber Grund bei forward move
#define bSingleStep  false

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Base class services

#ifdef _WINDOWS
CString CMouseCtrlBase::MsgToString(typDest dest, typCmd cmd, int val1, int val2, int val3)
{                 
    const char* scmd[] = { "SetMotorPos", "PosReached", "GetSensorValue", "SensorValue", "MoveLeg", "StepLeg", "StepDone", "InitMouse", "Trott", "StopAll",  "","" };
    const char* sdst[] = { "Spine ", "Ctrl    ", "LegFL", "LegFR", "LegHL", "LegHR", "Tail ", "Head ", "Body ","","","" };
    CString str;  str.Format(_T("%s %s %i, %i, %i"), CString(sdst[dest]), CString(scmd[cmd]), val1, val2, val3);
    return str;
}
#endif

// Sendfunktion für alle RPI-internen Objekte untereinander und Richtung UART
void CMouseCtrlBase::SendMsg(typDest dest, typCmd cmd, int val1, int val2, int val3)   
{  
    m_uReplies = 0;      // Empfangszähler rücksetzen

    if (dest==Spine) {   // und raus.
        LogMessage(MsgToString(dest, cmd, val1,val2,val3));    // Alles was rausgeht loggen
        ProcessSpine(cmd, val1, val2, val3);     // hier Windows-Simulation,   in echt dann die UART-Schnittstelle
    }
    else                 // sonst interne Kommunikation
        pRpi->DispatchMsg(dest, cmd, val1, val2, val3);
}  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Entry point Simulation- und UART-replies/events, 
void CRPI::ReceiveMsg(typCmd cmd, int val1, int val2, int val3) 
{   
    switch (cmd) {
    case InitMouse:    // von Cmd Shell
    case Trott:
    case StopAll:
        DispatchMsg(Ctrl, cmd, val1, val2, val3);
        break;

    case PosReached:   // die von Spine nur mit Cmd,
    case SensorValue:  //    hier ersma Receiver ermitteln
        DispatchMsg(LegFromMotId(val1), cmd, val1, val2, val3);
        break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Message dispatcher RPI internal between all rpi members
void CRPI::DispatchMsg(typDest dest, typCmd cmd, int val1, int val2, int val3)   
{
    LogMessage(MsgToString(dest, cmd, val1,val2,val3));     // log all received messages

    switch (dest) {
    case Ctrl:                    // hier ersma Blitztest, eigentlich über Ctrl-Modul oder Brause über alle Items
        TrottWalk.ProcessMsg(cmd, val1,val2,val3);  // v1 = time f. init, Trott
        break;

    case LegHL: case LegHR: case LegFR: case LegFL:
        LegFromDest(dest).ProcessMsg(cmd, val1,val2,val3);
        break;

    case Tail:   // Baustelle
    case Head:
    case Body:
        break;

        //case Spine: gibbshiernich
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Leg class state machine

void CMouseLeg::ProcessMsg(typCmd cmd, int val1, int val2, int val3)  // state machine dispatcher
{ 
    //switch (m_uState) {    // state machinery currently not in use
    //  case 0:
    switch (cmd) {
    case MoveLeg:   // direct move
        MoveTo(val1, val2, val3);  // x/y, time
        break;

    case StepLeg:   // stepped move
        StepStart(val1, val2, val3);
        break;

    case PosReached:
        if (++m_uReplies==m_uExpReplies &&  // beide Motoren abwarten
                !StepNext())                    // Achtung: das zieht auch bei MoveTo == single step!
            SendMsg(Ctrl, StepDone, pRpi->LegFromMotId(MotBase)) ; // Quit to CTrott
        break;

    case SensorValue:
        break;
    }
    //break;

    //case 1:
    //  switch (cmd) {
    //    case PosReached:
    //      if (++m_uReplies==m_uExpReplies)
    //        SetState(0);
    //    break;
    //  }
    //break;
    //}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// leg machine event methods

void CMouseLeg::MoveTo(double x, double y, int time)      // direkt single step to x/y, no trajectory, 
{ 
    vx = vy = 0;  step = stepcount = 0;   // muss, weil auch MausInit über PosReached StepNext aufruft!!
    jobTime = time;
    ptLeg = CKoord(x,y);
    SetPosition(NextWayPoint());
}

#ifdef alt  // ausführlich getestet, aber jeder Punkt bei Auftrag gerechnet
void CMouseLeg::StepStart(double x, double y, int time)   // step mode by trajectory
{ 
    step = 0;                      // cast: Abschneiden gewollt s. jobTime
    stepcount = (int)abs((long long)(x-ptLeg.x)/uStepResolution);  // nur x, die Maus läuft ja waagerecht.
    if (stepcount <= 1)
        MoveTo(x, y, time);          // if Dist < Resolution then Singlestep.
    else {
        jobTime = time/stepcount;    // jobtime in ms
        vx = (x-ptLeg.x)/stepcount;  // compute step vector
        vy = (y-ptLeg.y)/stepcount;
        StepNext();                  // first step of kinematik move
    }
}

bool CMouseLeg::StepNext() 
{
    if (++step > stepcount) return false;        // fertig
    CLegPos ang = NextWayPoint();
    SetPosition(ang);
    return true;
}

CLegPos CMouseLeg::NextWayPoint()
{
    ptLeg.x += vx;  ptLeg.y += vy;               // Vektor auf letzten Punkt addieren
    double X = ptLeg.x, Y = ptLeg.y;             // Umladen, weil ggf Rückweg höher
    if (vx<0 && step!=stepcount) Y += uPawLift;  // Rückweg; 1 cm anheben, letzter Step wieder runter
    return (MotBase%10 < 3) ? ikforeleg(X, Y, side)
                            : ikhindleg(X, Y, side);
}

void CMouseLeg::SetPosition(CLegPos ang) 
{ 
    m_uExpReplies = 2;                     // cast: 10ntel Grad reichen
    SendMsg(Spine, SetMotorPos, MotBase,   (int)ang.leg,  jobTime);  m_uExpReplies -= m_uReplies;  // falls rekursive Sofortantwort von SendMsg (nur Simulation!)
    SendMsg(Spine, SetMotorPos, MotBase+1, (int)ang.coil, jobTime);
}
#else

// Punkte werden im Voraus gerechnet (dgNext), während der Motor läuft, besser
void CMouseLeg::StepStart(double x, double y, int time)   // step mode by trajectory
{ 
    step = 0;                              // cast: Abschneiden gewollt s. jobTime
    stepcount = (int)std::abs((long long)(x-ptLeg.x)/uStepResolution);  // nur x, die Maus läuft ja waagerecht.
    if (stepcount <= 1)
        MoveTo(x, y, time);                  // if Dist < Resolution then Singlestep.
    else {
        jobTime = time/stepcount;            // jobtime in ms
        vx = (x-ptLeg.x)/stepcount;          // compute step vector from known position (!)
        vy = (y-ptLeg.y)/stepcount;
        dgNext = NextWayPoint();             // first step of kinematik move
        StepNext();                          // ... ausgeben und neuen berechnen
    }
}

bool CMouseLeg::StepNext() 
{
    if (++step > stepcount) return false;  // fertig
    SetPosition(dgNext);                   // vorausberechneten Punkt ausgeben
    if (step < stepcount)                  // wenn nicht letzter Punkt:
        dgNext = NextWayPoint();             //   neuen Punkt berechnen, solange Motor läuft
    return true;                           // weiter gehts
}

CLegPos CMouseLeg::NextWayPoint()
{    
    double X = ptLeg.x+vx, Y = ptLeg.y+vy;        // Nächsten Punkt ab current ptLeg errechnen
    if (vx<0 && step<stepcount-1) Y += uPawLift;  // Rückweg; 1 cm anheben, letzter Step wieder runter
    return (MotBase%10 < 3) ? ikforeleg(X, Y, side)
                            : ikhindleg(X, Y, side);
}

void CMouseLeg::SetPosition(CLegPos ang) 
{ 
    m_uExpReplies = 2;                  // cast: 10ntel Grad reichen
    SendMsg(Spine, SetMotorPos, MotBase,   (int)ang.leg,  jobTime);  m_uExpReplies -= m_uReplies;  // falls rekursive Sofortantwort von SendMsg (nur Simulation!)
    SendMsg(Spine, SetMotorPos, MotBase+1, (int)ang.coil, jobTime);
    ptLeg.x += vx;  ptLeg.y += vy;      // Vektor auf letzten Punkt addieren
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walking class trott

static bool vorne=false;
void CTrott::ProcessMsg(typCmd cmd, int val1, int val2, int val3)
{
    switch (cmd) {
    case InitMouse:            // rechtes Vorderbein vorne
        SendMsg(LegHL, MoveLeg,            uHindLegStart,  uWalkLevel, val1);
        SendMsg(LegHR, MoveLeg, uStepLengthH+uHindLegStart,  uWalkLevel, val1);
        SendMsg(LegFL, MoveLeg, uStepLengthF+uFrontLegStart, uWalkLevel, val1);
        SendMsg(LegFR, MoveLeg,            uFrontLegStart, uWalkLevel, val1);
        m_uExpReplies=40;  vorne=true;   // 40: damit StepDone nicht zum Schrittwechsel führt...!
        break;

    case Trott:
        uPaceTime = val1;
        if (vorne) {             // Schritt linkes Vorderbein vor
            SendMsg(LegHL, StepLeg, uStepLengthH+uHindLegStart,  uWalkLevel, uPaceTime);
            SendMsg(LegHR, StepLeg,            uHindLegStart,  uWalkLevel, uPaceTime);
            SendMsg(LegFL, StepLeg,            uFrontLegStart, uWalkLevel, uPaceTime);
            SendMsg(LegFR, StepLeg, uStepLengthF+uFrontLegStart, uWalkLevel, uPaceTime);
            m_uExpReplies=4;
            vorne=false;
        } else {                 // Schritt rechtes Vorderbein vor
            SendMsg(LegFL, StepLeg, uStepLengthF+uFrontLegStart, uWalkLevel, uPaceTime);
            SendMsg(LegFR, StepLeg,            uFrontLegStart, uWalkLevel, uPaceTime);
            SendMsg(LegHL, StepLeg,            uHindLegStart,  uWalkLevel, uPaceTime);
            SendMsg(LegHR, StepLeg, uStepLengthH+uHindLegStart,  uWalkLevel, uPaceTime);
            m_uExpReplies=4;  vorne=true;
        }
        break;

    case StepDone:
        if (++m_uReplies==m_uExpReplies)
            if (!bSingleStep) ProcessMsg(Trott, uPaceTime);   // Schrittwechsel
        break;

    case StopAll:
        SendMsg(Spine, cmd);
        break;
    }
}

// finito basta
//////////////////////// <>  ||
