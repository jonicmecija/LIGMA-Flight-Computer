/*!
 *  @file stateMachine.h
 *
 *  State machine library for the LIGMA rocket flight computer.
 *
 *  Jonic Mecija
 *  November 6, 2022
 */

/* State Transition Functions */
bool detectLaunch(float currAltitude);
bool detectApogee(float currAltitude);
bool detectLanding(float currAltitude);