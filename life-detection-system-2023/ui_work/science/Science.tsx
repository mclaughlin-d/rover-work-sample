import ActuatorFrame from './actuatorFrame';
import LightFrame from './lightFrame';
import PumpFrame from './pumpFrame';

export default function Science() {
  const ldCamStream1 = ''; // ADD CAMERA STREAM USB HERE
  return (
    <div className="flex flex-col gap-6 p-6 sm:h-full sm:flex-row">
      <div className="relative h-full w-full">
        <div className="flex flex-col justify-center gap-6">
          <ActuatorFrame />
          <PumpFrame />
        </div>
      </div>
      <div className="grid w-full max-w-xl grid-cols-1 content-start gap-4">
        <LightFrame />
      </div>
    </div>
  );
}

