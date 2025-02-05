import useRosTopic from '../core/hooks/useRosTopic';
import topics from '../topics.yaml';
import Frame from '../core/frames/Frame';
import LightSensor from './lightSensor';
import DriverTimer from '../driver/DriverTimer';
import PulseLight from '../core/readouts/PulseLight';
import { useEffect, useRef } from 'react';
import { Status } from '../core/colors';

export default function LightFrame() {
  const { data: lightSensNum } = useRosTopic(topics.ld.lightCmd);
  const { data: lastLightCmd } = useRosTopic(topics.ld.lightRequest);
  const { data: lightSensRead } = useRosTopic(topics.ld.lightReading); // just dislpay for now

  const lightNums = [0, 1, 2, 3, 4, 5];

  const lightReq = useRef<any>();
  useEffect(() => {
    lightReq.current?.pulse(Status.OK);
  }, [useRosTopic(topics.ld.lightRequest)]);

  return (
    <Frame className="w-full h-full">
      <div className="flex flex-col gap-2 items-center">
        <span className="font-bold text-xl">Light Sensors</span>
        <div className="flex flex-row gap-6 items-center">
          <div className="flex flex-col gap-2">
            <span>
              Light sensor selected:{' '}
              {isNaN(lightSensNum) ? '' : lightSensNum + 1}
            </span>
            <span>
              Last sensor request sent:{' '}
              {isNaN(lastLightCmd) ? '' : lastLightCmd + 1}
            </span>
            <span>
              Light sensor reading published:{' '}
              {isNaN(lightSensRead) ? '' : lightSensRead.toFixed(8)}
            </span>
          </div>
          <PulseLight ref={lightReq} />
        </div>
        {lightNums.map((num) => (
          <LightSensor lightNum={num} />
        ))}
        <DriverTimer />
      </div>
    </Frame>
  );
}

