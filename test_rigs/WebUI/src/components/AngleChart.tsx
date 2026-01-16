import React, { useState, useEffect, useRef } from 'react';
import {
  Chart as ChartJS,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  ChartOptions,
} from 'chart.js';
import { Line } from 'react-chartjs-2';
import ConnManager from '../ConnManager';

ChartJS.register(
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

const connManager = ConnManager.getInstance();

interface AngleChartProps {
  lastUpdate: number;
}

interface DataPoint {
  timestamp: number;
  mt6701: number | null;
  as5600: number | null;
}

const MAX_DATA_POINTS = 200;

export default function AngleChart({ lastUpdate }: AngleChartProps) {
  const [dataHistory, setDataHistory] = useState<DataPoint[]>([]);
  const startTimeRef = useRef<number | null>(null);

  useEffect(() => {
    const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
    if (!deviceManager) return;

    let mt6701Angle: number | null = null;
    let as5600Angle: number | null = null;
    let timestamp: number = Date.now();

    // Get MT6701 encoder data
    const mt6701State = deviceManager.getDeviceState('I2CA_6_MT6701');
    if (mt6701State?.deviceAttributes?.angle) {
      const values = mt6701State.deviceAttributes.angle.values;
      if (values.length > 0) {
        mt6701Angle = values[values.length - 1];
        // Use the timestamp from the device timeline if available
        const timestamps = mt6701State.deviceTimeline?.timestampsUs;
        if (timestamps && timestamps.length > 0) {
          // Convert microseconds to milliseconds
          timestamp = timestamps[timestamps.length - 1] / 1000;
        }
      }
    }

    // Get AS5600 encoder data
    const as5600State = deviceManager.getDeviceState('I2CA_36_AS5600');
    if (as5600State?.deviceAttributes?.angle) {
      const values = as5600State.deviceAttributes.angle.values;
      if (values.length > 0) {
        as5600Angle = values[values.length - 1];
        // If we didn't get timestamp from MT6701, use AS5600's timestamp
        if (timestamp === Date.now()) {
          const timestamps = as5600State.deviceTimeline?.timestampsUs;
          if (timestamps && timestamps.length > 0) {
            // Convert microseconds to milliseconds
            timestamp = timestamps[timestamps.length - 1] / 1000;
          }
        }
      }
    }

    // Only add data point if we have at least one valid angle
    if (mt6701Angle !== null || as5600Angle !== null) {
      setDataHistory((prev) => {
        const newData = [
          ...prev,
          {
            timestamp: timestamp,
            mt6701: mt6701Angle,
            as5600: as5600Angle,
          },
        ];

        // Set start time reference to the first data point's timestamp
        if (startTimeRef.current === null && newData.length > 0) {
          startTimeRef.current = newData[0].timestamp;
        }

        // Keep only the last MAX_DATA_POINTS
        if (newData.length > MAX_DATA_POINTS) {
          return newData.slice(newData.length - MAX_DATA_POINTS);
        }
        return newData;
      });
    }
  }, [lastUpdate]);

  const chartData = {
    datasets: [
      {
        label: 'MT6701',
        data: dataHistory.map((point) => {
          const startTime = startTimeRef.current ?? (dataHistory.length > 0 ? dataHistory[0].timestamp : point.timestamp);
          const elapsed = (point.timestamp - startTime) / 1000;
          return {
            x: elapsed,
            y: point.mt6701
          };
        }),
        borderColor: 'rgb(74, 158, 255)',
        backgroundColor: 'rgba(74, 158, 255, 0.1)',
        borderWidth: 2,
        pointRadius: 0,
        pointHitRadius: 10,
        tension: 0.1,
        spanGaps: true,
      },
      {
        label: 'AS5600',
        data: dataHistory.map((point) => {
          const startTime = startTimeRef.current ?? (dataHistory.length > 0 ? dataHistory[0].timestamp : point.timestamp);
          const elapsed = (point.timestamp - startTime) / 1000;
          return {
            x: elapsed,
            y: point.as5600
          };
        }),
        borderColor: 'rgb(255, 99, 132)',
        backgroundColor: 'rgba(255, 99, 132, 0.1)',
        borderWidth: 2,
        pointRadius: 0,
        pointHitRadius: 10,
        tension: 0.1,
        spanGaps: true,
      },
    ],
  };

  const options: ChartOptions<'line'> = {
    responsive: true,
    maintainAspectRatio: false,
    animation: {
      duration: 0, // Disable animation for real-time performance
    },
    scales: {
      x: {
        type: 'linear',
        title: {
          display: true,
          text: 'Time (s)',
          color: '#888',
        },
        ticks: {
          color: '#888',
          maxTicksLimit: 10,
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.1)',
        },
      },
      y: {
        title: {
          display: true,
          text: 'Angle (°)',
          color: '#888',
        },
        ticks: {
          color: '#888',
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.1)',
        },
        min: 0,
        max: 360,
      },
    },
    plugins: {
      legend: {
        display: true,
        position: 'top',
        labels: {
          color: '#e0e0e0',
          usePointStyle: true,
        },
      },
      tooltip: {
        mode: 'index',
        intersect: false,
        callbacks: {
          label: function (context) {
            let label = context.dataset.label || '';
            if (label) {
              label += ': ';
            }
            if (context.parsed.y !== null) {
              label += context.parsed.y.toFixed(1) + '°';
            }
            return label;
          },
        },
      },
    },
    interaction: {
      mode: 'nearest',
      axis: 'x',
      intersect: false,
    },
  };

  return (
    <div className="panel chart-panel">
      <h2>Angle History</h2>
      <div className="chart-info">
        <small className="text-muted">
          Real-time data · {dataHistory.length}/{MAX_DATA_POINTS} points
        </small>
      </div>
      <div className="chart-container">
        <Line data={chartData} options={options} />
      </div>
    </div>
  );
}
