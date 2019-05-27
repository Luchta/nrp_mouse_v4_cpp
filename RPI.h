
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RPI.h: Schnittstelle der Maus- Steuerungs class
//

#pragma once

#include "kinematics.h"
#ifdef _WINDOWS
class mouse_com {
public: mouse_com() {}
  typedef enum Commmands{ SetMotorPos, PosReached, GetSensorValue, SensorValue,   // commands spine
                          MoveLeg, StepLeg, StepDone,                             // commands rpi internal
                          InitMouse, Trott, StopAll } typCmd;                     // commands from shell
};

#else
#include "Mouse_COM.h"
#endif

class CMousCtrlSet {
public:
  CMousCtrlSet() {
    uStepResolution = 14;  // length of kinematic-step   (alles in mm)
    uFrontLegStart  = -10;  // x start pos. of pace
    uHindLegStart   = -20;
    uStepLengthF    = 70;  // length of one leg move on the ground
    uStepLengthH    = 70;  // length of one leg move on the ground
    uWalkLevel      = 10;  // y walking level of init and walking 
    uPawLift        = 10;  // Höhe über Grund bei forward move
    bSingleStep = false;
  }

  int  uStepResolution, uStepLengthH, uStepLengthF,
       uFrontLegStart,  uHindLegStart, 
       uWalkLevel,      uPawLift;       
  bool bSingleStep;
};

class CRPI;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Basisklasse für alle: Msg-Verkehr, Logging, StateCtrl
class CMouseCtrlBase : public CMouseCom
{
public:
  CMouseCtrlBase()  { m_uState=0; } //CMouseCtrlBase(CRPI& r) : rpi(r) { m_uState=0;  }
  virtual ~CMouseCtrlBase() {}

public:
  typedef enum Destination{ Spine, Ctrl, LegFL, LegFR, LegHL, LegHR, Tail, Head, Body } typDest;

protected:
  void SendMsg(typDest dest, typCmd cmd, int val1=0, int val2=0, int val3=0);  
#ifdef _WINDOWS
  CString MsgToString(typDest dest, typCmd cmd, int val1, int val2, int val3);
#else 
  int MsgToString(typDest, typCmd, int, int, int) { return 0; }
  void LogMessage(int) {}
#endif

  void SetState(int state) { m_uState=state; }
  int m_uState, m_uReplies, m_uExpReplies;
  CRPI* pRpi; // CRPI& rpi;
};

#ifdef _WINDOWS
extern void ProcessSpine(CMouseCtrlBase::typCmd cmd, int val1=0, int val2=0, int val3=0);   // nur f. Simulation, in echt über UART, s. CMouseCtrlBase::SendMsg
extern void LogMessage(CString str);
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// class for the 4 legs
class CMouseLeg : public CMouseCtrlBase, public CKinematics
{
public:
  CMouseLeg() { jobTime=500; vx=vy=0; }  //CMouseLeg(CRPI& r) : CMouseCtrlBase(r) { jobTime=500; vx=vy=0; } 
  void SetType(int mb, CRPI* prpi) { MotBase = mb; pRpi=prpi; side = MotBase < 20 ? 'l':'r'; }
  virtual ~CMouseLeg() {}

public:
  void ProcessMsg(typCmd cmd, int val1=0, int val2=0, int val3=0);

private:
  void MoveTo   (double x, double y, int time=400);
  void StepStart(double x, double y, int time=500);
  bool StepNext ();
  CLegPos NextWayPoint();
  void SetPosition(CLegPos ang);//double deg1, double deg2);

  CKoord  ptLeg;   // x/y destination pos in move, current pos !in move
  CLegPos dgNext;  // Next Point on walking line
  double vx, vy;   // vector from current to destination x/y-point, divided by step stepcount
  int  stepcount,  // number of kinematic-steps
       step,       // current step number
       jobTime,    // time duration of one kinematic step
       MotBase;    // Motor id of hip motor (knee++)
  char side;   
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// class to command the legs: Trott, Init, Stop
class CTrott : public CMouseCtrlBase 
{
public:
  CTrott() {}
  void Init(CRPI* prpi) { pRpi=prpi; }  
  virtual ~CTrott() {}
public:
  void ProcessMsg(typCmd cmd, int val1=0, int val2=0, int val3=0);

private:
  int uPaceTime;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main module of RPI Software
class CRPI : public CMouseCtrlBase
{
  friend CMouseCtrlBase;  // damit der den DispatchMsg verwenden darf, aber außerhalb niemand!

// construction
public:
  CRPI() {}
  CMousCtrlSet& InitRPI() {
    pRpi = this; //der eigene pRpi -Pointer sollt  ssh. auch besetzt sein.
    HindLegL.SetType(13, this);  HindLegR.SetType(23, this);  TrottWalk.Init(this);
    ForeLegR.SetType(21, this);  ForeLegL.SetType(11, this);  return mcs; } 
  virtual ~CRPI() {}

// public msg entrypoint
public:
  virtual void ReceiveMsg(typCmd cmd, int val1=0, int val2=0, int val3=0);

#ifdef _WINDOWS      // only for Simulation
  void getHindLegDims(double& h1, double& h2, double& h3, double& h3i) { HindLegL.getHindLegDims(h1,h2,h3,h3i); }  
  void getForeLegDims(double& h1, double& h2, double& h3, double& h3i) { HindLegL.getForeLegDims(h1,h2,h3,h3i); } 
#endif

protected:           // der MsgDispatcher verwendet auch von CMouseCtrlBase::SendMsg, deshalb hier Friend
  void DispatchMsg(typDest dest, typCmd cmd, int val1=0, int val2=0, int val3=0);   

public:
  Destination LegFromMotId(int id) { switch (id) { case 11:case 12: return LegFL; case 13:case 14: return LegHL; case 21:case 22: return LegFR; case 23:case 24: return LegHR; return (Destination)-1; } }

  CMousCtrlSet mcs;

private:
  CMouseLeg& LegFromDest(int dest) { switch (dest) { case LegFL: return ForeLegL; case LegFR: return ForeLegR; case LegHL: return HindLegL; default: return HindLegR; } }
  
  CMouseLeg HindLegL, HindLegR, ForeLegL, ForeLegR;
  CTrott    TrottWalk;

};

