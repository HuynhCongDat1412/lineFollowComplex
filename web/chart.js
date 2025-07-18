let fullLinePos = [];
let fullCorrection = [];
let currentIndex = 0;
let maxPoints = 500;

let chart;
function setupChart() {
    const ctx = document.getElementById('chartCanvas').getContext('2d');

    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Line Pos',
                    data: [],
                    yAxisID: 'y1',
                    borderColor: 'blue',
                    backgroundColor: 'rgba(0,0,255,0.1)',
                    borderWidth: 2,
                    pointRadius: 2,
                    tension: 0.2
                },
                {
                    label: 'Correction',
                    data: [],
                    yAxisID: 'y2',
                    borderColor: 'red',
                    backgroundColor: 'rgba(255,0,0,0.1)',
                    borderWidth: 2,
                    pointRadius: 2,
                    tension: 0.2
                }
            ]
        },
        options: {
            responsive: true,
            animation: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Time (tick)'
                    }
                },
                y1: {
                    type: 'linear',
                    position: 'left',
                    title: {
                        display: true,
                        text: 'Line Position'
                    },
                    min: -2.5,
                    max: 2.5
                },
                y2: {
                    type: 'linear',
                    position: 'right',
                    title: {
                        display: true,
                        text: 'Correction'
                    },
                    min: -100,
                    max: 100,
                    grid: {
                        drawOnChartArea: false
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top'
                },
                zoom: {
                    pan: {
                        enabled: true,
                        mode: 'x',
                        modifierKey: 'ctrl'
                    },
                    zoom: {
                        wheel: {
                            enabled: true
                        },
                        pinch: {
                            enabled: true
                        },
                        mode: 'x'
                    }
                }
            }
        }
    });
}


function updateChart(linePos, correction) {
    // Lưu vào buffer chính
    fullLinePos.push(linePos);
    fullCorrection.push(correction);
    currentIndex = fullLinePos.length - 1;

    // Giới hạn chiều dài buffer
    if (fullLinePos.length > maxPoints) {
        fullLinePos.shift();
        fullCorrection.shift();
        currentIndex--;
    }

    // Cập nhật slider
    const slider = document.getElementById('chartSlider');
    slider.max = fullLinePos.length - 1;
    slider.value = currentIndex;

    // Cắt ra đoạn để vẽ
    const start = Math.max(0, currentIndex - 50);
    const end = currentIndex;

    const labels = Array.from({ length: end - start + 1 }, (_, i) => start + i);
    const lineData = fullLinePos.slice(start, end + 1);
    const correctionData = fullCorrection.slice(start, end + 1);

    chart.data.labels = labels;
    chart.data.datasets[0].data = lineData;
    chart.data.datasets[1].data = correctionData;
    chart.update();
}

