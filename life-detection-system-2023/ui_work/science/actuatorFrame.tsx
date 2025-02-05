import Scale from '../core/readouts/Scale';
import useRosTopic from '../core/hooks/useRosTopic';
import topics from '../topics.yaml';
import Frame from '../core/frames/Frame';
import { useState, useEffect } from 'react';

export default function ActuatorFrame() {
  const { data: actCmd } = useRosTopic(topics.ld.actCmd);
  const { data: actuatorPosn } = useRosTopic(topics.ld.actuator);
  const { data: actCmdMode } = useRosTopic(topics.ld.ctrlMode);

  const [ctrlMode, setCtrlMode] = useState('Position');

  useEffect(() => {
    if (actCmdMode) {
      setCtrlMode('Position');
    } else if (actCmdMode === false) {
      setCtrlMode('Velocity');
    } else {
      setCtrlMode('Position');
    }
  }, [actCmdMode]);

  return (
    <Frame className="flex flex-col gap-3">
      <span className="font-bold text-xl self-center">Actuator</span>
      <span>Actuator control mode: {ctrlMode}</span>
      <span>
        Last actuator position command: {isNaN(actCmd) ? '' : actCmd + 1}
      </span>
      <Scale
        label={'Actuator Position'}
        min={0.0}
        max={1.0}
        roundTo={2}
        value={actuatorPosn}
      />
    </Frame>
  );
}

