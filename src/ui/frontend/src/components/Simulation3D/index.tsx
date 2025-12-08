import React, { useEffect, useRef, useState, useCallback } from 'react';
import { IonCard, IonCardContent, IonCardHeader, IonCardTitle, IonButton, IonLabel } from '@ionic/react';

interface SimulationData {
  t: number;
  north: number;
  east: number;
  down: number;
  roll: number;
  pitch: number;
  yaw: number;
}

const Simulation3D: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number | null>(null);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  
  const [isRunning, setIsRunning] = useState(false);
  const [simulationData, setSimulationData] = useState<SimulationData>({
    t: 0,
    north: 0,
    east: 0,
    down: 0,
    roll: 0,
    pitch: 0,
    yaw: 0
  });

  // Canvas drawing function
  const drawSimulation = useCallback(() => {
    if (!canvasRef.current) return;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = '#87CEEB'; // Sky blue
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Set up coordinate system (center origin)
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const scale = 20; // pixels per meter

    ctx.save();
    ctx.translate(centerX, centerY);

    // Draw grid
    ctx.strokeStyle = '#ccc';
    ctx.lineWidth = 1;
    for (let i = -10; i <= 10; i++) {
      // Vertical lines
      ctx.beginPath();
      ctx.moveTo(i * scale * 2, -canvas.height / 2);
      ctx.lineTo(i * scale * 2, canvas.height / 2);
      ctx.stroke();
      
      // Horizontal lines  
      ctx.beginPath();
      ctx.moveTo(-canvas.width / 2, i * scale * 2);
      ctx.lineTo(canvas.width / 2, i * scale * 2);
      ctx.stroke();
    }

    // Draw coordinate axes
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 2;
    // X axis (East)
    ctx.strokeStyle = '#0000ff';
    ctx.beginPath();
    ctx.moveTo(-50, 0);
    ctx.lineTo(50, 0);
    ctx.stroke();
    // Y axis (North, inverted for screen coordinates)  
    ctx.strokeStyle = '#00ff00';
    ctx.beginPath();
    ctx.moveTo(0, -50);
    ctx.lineTo(0, 50);
    ctx.stroke();

    // Draw quadcopter
    const quadX = -simulationData.east * scale; // East -> -X
    const quadY = simulationData.north * scale;  // North -> Y (inverted)
    
    ctx.save();
    ctx.translate(quadX, quadY);
    ctx.rotate(simulationData.yaw);

    // Quadcopter body
    ctx.fillStyle = '#333';
    ctx.fillRect(-15, -15, 30, 30);

    // Direction indicator (red triangle)
    ctx.fillStyle = '#ff0000';
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(5, -8);
    ctx.lineTo(5, 8);
    ctx.closePath();
    ctx.fill();

    // Arms (simple lines)
    ctx.strokeStyle = '#666';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(-20, -20);
    ctx.lineTo(20, 20);
    ctx.moveTo(-20, 20);
    ctx.lineTo(20, -20);
    ctx.stroke();

    // Propellers
    ctx.fillStyle = '#888';
    const propPositions = [[-20, -20], [20, -20], [20, 20], [-20, 20]];
    propPositions.forEach(([x, y]) => {
      ctx.beginPath();
      ctx.arc(x, y, 8, 0, 2 * Math.PI);
      ctx.fill();
    });

    ctx.restore();
    ctx.restore();
  }, [simulationData]);

  // Fetch simulation data from Flask backend
  const fetchSimulationData = async () => {
    try {
      const response = await fetch('http://localhost:5000/api/simulation/step');
      const data = await response.json();
      setSimulationData({
        t: data.t,
        north: data.nord,
        east: data.ost,
        down: data.unten,
        roll: data.roll,
        pitch: data.pitch,
        yaw: data.yaw
      });
    } catch (error) {
      console.error('Error fetching simulation data:', error);
    }
  };

  // Animation loop
  const animate = () => {
    drawSimulation();
    if (isRunning) {
      animationRef.current = requestAnimationFrame(animate);
    }
  };

  // Start/stop simulation
  const toggleSimulation = () => {
    if (isRunning) {
      setIsRunning(false);
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    } else {
      setIsRunning(true);
      animate();
      // Start fetching data
      intervalRef.current = setInterval(() => {
        fetchSimulationData();
      }, 50); // 20 FPS
    }
  };

  // Initialize canvas
  useEffect(() => {
    if (canvasRef.current) {
      canvasRef.current.width = 600;
      canvasRef.current.height = 400;
      drawSimulation();
    }
  }, [drawSimulation]);

  // Redraw when data changes
  useEffect(() => {
    drawSimulation();
  }, [simulationData, drawSimulation]);

  return (
    <IonCard style={{ height: '500px', margin: '10px' }}>
      <IonCardHeader>
        <IonCardTitle>2D Quadcopter Simulation</IonCardTitle>
      </IonCardHeader>
      <IonCardContent>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '10px' }}>
          <IonButton onClick={toggleSimulation} color={isRunning ? 'danger' : 'success'}>
            {isRunning ? 'Stop' : 'Start'} Simulation
          </IonButton>
          <div>
            <IonLabel>T: {simulationData.t.toFixed(2)}s</IonLabel>
            <br />
            <IonLabel>Pos: N:{simulationData.north.toFixed(2)}, E:{simulationData.east.toFixed(2)}, D:{simulationData.down.toFixed(2)}</IonLabel>
          </div>
        </div>
        <canvas 
          ref={canvasRef} 
          style={{ 
            width: '100%', 
            height: '400px', 
            border: '1px solid #ddd',
            borderRadius: '8px',
            backgroundColor: '#87CEEB'
          }} 
        />
      </IonCardContent>
    </IonCard>
  );
};

export default Simulation3D;