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

#define uStepResolution  pRpi->mcs.uStepResolution
#define uFrontLegStart   pRpi->mcs.uFrontLegStart 
#define uHindLegStart    pRpi->mcs.uHindLegStart  
#define uStepWidth       pRpi->mcs.uStepWidth     
#define uWalkLevel       pRpi->mcs.uWalkLevel     
#define uPawLift         pRpi->mcs.uPawLift       
#define bSingleStep      pRpi->mcs.bSingleStep

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

//UART-Schnittstelle
void ProcessSpine(CMouseCtrlBase::typCmd cmd, int val1, int val2, int val3)
{

}

// Sendfunktion f�r alle RPI-internen Objekte untereinander und Richtung UART
void CMouseCtrlBase::SendMsg(typDest dest, typCmd cmd, int val1, int val2, int val3)   
{  
  m_uReplies = 0;      // Empfangsz�hler r�cksetzen

  if (dest==Spine) {   // und raus.
    //LogMessage(MsgToString(dest, cmd, val1,val2,val3));    // Alles was rausgeht loggen
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
    case Sensor_Value: //    hier ersma Receiver ermitteln
      DispatchMsg(LegFromMotId(val1), cmd, val1, val2, val3); 
    break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Message dispatcher RPI internal between all rpi members
void CRPI::DispatchMsg(typDest dest, typCmd cmd, int val1, int val2, int val3)   
{
  //LogMessage(MsgToString(dest, cmd, val1,val2,val3));     // log all received messages

  switch (dest) {
    case Ctrl:                    // hier ersma Blitztest, eigentlich �ber Ctrl-Modul oder Brause �ber alle Items
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

        case StepLeg:   // From MousCtrl: val1 war bein (0-3)
          StepStart(val1, val2, val3); 
        break;

        case PosReached:
          if (++m_uReplies==m_uExpReplies) {
            if (!StepNext())
              SendMsg(Ctrl, StepDone, pRpi->LegFromMotId(MotBase)) ; // Quit to MousWalk
          }
        break;

        case Sensor_Value:
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
  vx = vy = 0;  //step = 0; stepcount = 1; 
  jobTime = time;  
  ptLeg = CKoord(x,y);       
  SetPosition(NextWayPoint()); 
}

void CMouseLeg::StepStart(double x, double y, int time)   // step mode by trajectory
{ 
  step = 0;                      // cast: Abschneiden gewollt s. jobTime
  stepcount = (int)abs((long long)(x-ptLeg.x)/uStepResolution);  // nur x, die Maus l�uft ja waagerecht.
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
  double X = ptLeg.x, Y = ptLeg.y;             // Umladen, weil ggf R�ckweg h�her                  
  if (vx<0 && step!=stepcount) Y += uPawLift;  // R�ckweg; 1 cm anheben, letzter Step wieder runter
  return (MotBase%10 < 3) ? ikforeleg(X, Y, side) 
                          : ikhindleg(X, Y, side);
}

void CMouseLeg::SetPosition(CLegPos ang) 
{ 
  m_uExpReplies = 2;                  // cast: 10ntel Grad reichen
  SendMsg(Spine, SetMotorPos, MotBase,   (int)ang.leg,  jobTime);  m_uExpReplies -= m_uReplies;  // falls rekursive Sofortantwort von SendMsg (nur Simulation!)
  SendMsg(Spine, SetMotorPos, MotBase+1, (int)ang.coil, jobTime);      
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walking class trott

static bool vorne=false;
void CTrott::ProcessMsg(typCmd cmd, int val1, int val2, int val3)
{
  switch (cmd) {
    case InitMouse:
      SendMsg(LegHL, MoveLeg,            uHindLegStart,  uWalkLevel, val1);  
      SendMsg(LegHR, MoveLeg, uStepWidth+uHindLegStart,  uWalkLevel, val1); 
      SendMsg(LegFL, MoveLeg, uStepWidth+uFrontLegStart, uWalkLevel, val1); 
      SendMsg(LegFR, MoveLeg,            uFrontLegStart, uWalkLevel, val1);  
      m_uExpReplies=40;  vorne=true;
    break;

    case Trott:
      uPaceTime = val1;
      if (vorne) {
        SendMsg(LegHL, StepLeg, uStepWidth+uHindLegStart,  uWalkLevel, uPaceTime); 
        SendMsg(LegHR, StepLeg,            uHindLegStart,  uWalkLevel, uPaceTime);
        SendMsg(LegFL, StepLeg,            uFrontLegStart, uWalkLevel, uPaceTime);
        SendMsg(LegFR, StepLeg, uStepWidth+uFrontLegStart, uWalkLevel, uPaceTime);  
        m_uExpReplies=4;  vorne=false; 
      } else {                         
        SendMsg(LegFL, StepLeg, uStepWidth+uFrontLegStart, uWalkLevel, uPaceTime);
        SendMsg(LegFR, StepLeg,            uFrontLegStart, uWalkLevel, uPaceTime); 
        SendMsg(LegHL, StepLeg,            uHindLegStart,  uWalkLevel, uPaceTime);
        SendMsg(LegHR, StepLeg, uStepWidth+uHindLegStart,  uWalkLevel, uPaceTime);
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


