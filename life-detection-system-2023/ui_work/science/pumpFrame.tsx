import useRosTopic from '../core/hooks/useRosTopic';
import topics from '../topics.yaml';
import Frame from '../core/frames/Frame';
import { useState, useEffect } from 'react';

export default function PumpFrame() {
  const { data: pumpCmd } = useRosTopic(topics.ld.pumpCmd);
  const [pumpOn, setPumpOn] = useState('OFF');

  useEffect(() => {
    if (pumpCmd === 0) {
      setPumpOn('OFF');
    } else if (pumpCmd === 1) {
      setPumpOn('ON');
    } else if (pumpCmd === -1){
      setPumpOn('REVERSE');
    } else if (typeof pumpCmd === 'undefined') {
      setPumpOn('not published yet');
    } else {
      setPumpOn('recieving an invalid command');
    }
  }, [pumpCmd]);

  return (
    <Frame className="w-full h-full">
      <div className="flex flex-col gap-3">
        <span className="font-bold text-xl self-center">Water pump</span>
        <span>Current pump command: {pumpCmd}</span>
        <span>
          The pump is <span className="font-bold">{pumpOn}</span>
        </span>
      </div>
    </Frame>
  );
}

