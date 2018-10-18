/*
 * For the CPP implementation, there are many types that must be defined in
   order to integrate with the Python implementation. some of them are defined
   here.

 */

 namespace DRIVR {

 struct Args {
     float focal;
     float carW;
     float cameraH;
     float cameraMinAngle;
     float cameraMaxHorizAngle;
     float horizon;
 };

 struct Box {
     float left;
     float right;
     float top;
     float bottom;
 };

} // end DRIVR
