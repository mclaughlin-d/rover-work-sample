import useRosTopic from '../core/hooks/useRosTopic';
import topics from '../topics.yaml';
import { useState, useEffect } from 'react';

interface LightSensProps {
  lightNum: number;
}

export default function LightSensor({ lightNum }: LightSensProps) {
  const { data: lightSensNum } = useRosTopic(topics.ld.lightRequest);
  const { data: lightSensRead } = useRosTopic(topics.ld.lightReading);
  const [reading1, setReading1] = useState(0.0);
  const [reading2, setReading2] = useState(0.0);
  const [errMsg, setErrMsg] = useState('');
  const [reading, setReading] = useState(false);

  // whenever the light sensor number changes, change the reading values
  useEffect(() => {
    if (lightSensNum === lightNum) {
      setReading(true);
      // clear readings because new readings were requested - prob won't happen but still
      setReading1(0.0);
      setReading2(0.0);
      setErrMsg('');
    } else {
      setReading(false);
    }
  }, [lightSensNum]);

  // when the reading value changes, set either the first or the second value
  // new idea - use times, test if time is about two minutes after?? idk
  useEffect(() => {
    if (reading) {
      if (reading1 === 0.0) {
        setReading1(lightSensRead.toFixed(8));
      } else if (reading2 === 0.0) {
        setReading2(lightSensRead.toFixed(8));
      } else {
        setErrMsg('Two readings already been requested!');
        setReading(false);
      }
    }
  }, [lightSensRead]);

  return (
    <div className="flex gap-6 items-center rounded-[20px] border-2 p-4 w-full">
      <span className="font-bold text-lg">{lightNum + 1}</span>
      <div className="flex flex-col">
        <div className="flex justify-start gap-8">
          <span>Reading 1: {reading1}</span>
          <span>Reading 2: {reading2}</span>
        </div>
        <span>
          Difference between readings:{' '}
          {reading2 === 0 ? '' : reading2 - reading1}
        </span>
        <span>{errMsg}</span>
      </div>
    </div>
  );
}

