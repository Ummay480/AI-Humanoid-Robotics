#!/usr/bin/env python3
"""
Monitoring Dashboard Runner

Runs a simple monitoring dashboard for the perception system.
Displays real-time metrics, system health, and performance statistics.

Usage:
    python3 scripts/run_monitoring_dashboard.py
    python3 scripts/run_monitoring_dashboard.py --config config/monitoring_dashboard.yaml
    python3 scripts/run_monitoring_dashboard.py --web-server --port 8080
"""

import argparse
import time
import sys
import os
from pathlib import Path
import yaml
from typing import Dict, Any
import threading

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.perception.monitoring.logger import (
    get_performance_monitor,
    get_health_monitor
)
from src.perception.monitoring.performance_optimizer import PerformanceOptimizer


class MonitoringDashboard:
    """Simple terminal-based monitoring dashboard."""

    def __init__(self, config_path: str = None):
        """Initialize dashboard."""
        self.config = self._load_config(config_path)
        self.running = False

        # Get monitoring instances
        self.perf_monitor = get_performance_monitor()
        self.health_monitor = get_health_monitor()
        self.optimizer = PerformanceOptimizer()

    def _load_config(self, config_path: str) -> Dict:
        """Load dashboard configuration."""
        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # Default configuration
            return {
                'dashboard': {
                    'refresh_rate': 1.0,
                    'name': 'Perception System Monitor'
                }
            }

    def clear_screen(self):
        """Clear terminal screen."""
        os.system('clear' if os.name == 'posix' else 'cls')

    def render_header(self):
        """Render dashboard header."""
        name = self.config.get('dashboard', {}).get('name', 'Perception Monitor')
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

        print("=" * 80)
        print(f"{name:^80}")
        print(f"{timestamp:^80}")
        print("=" * 80)

    def render_system_health(self):
        """Render system health panel."""
        print("\n┌─ SYSTEM HEALTH " + "─" * 62 + "┐")

        overall_health = self.health_monitor.get_overall_health()
        health_color = {
            'OK': '✓',
            'WARNING': '⚠',
            'ERROR': '✗',
            'CRITICAL': '✗',
            'UNKNOWN': '?'
        }.get(overall_health, '?')

        print(f"│ Overall Status: {health_color} {overall_health:<30}                    │")

        # Component health
        component_health = self.health_monitor.get_health()
        if component_health:
            print("│" + " " * 78 + "│")
            print("│ Components:                                                              │")
            for component, status in list(component_health.items())[:5]:  # Show top 5
                status_symbol = {
                    'OK': '✓',
                    'WARNING': '⚠',
                    'ERROR': '✗',
                    'CRITICAL': '✗'
                }.get(status.get('status', 'UNKNOWN'), '?')
                component_name = component[:25].ljust(25)
                status_text = status.get('status', 'UNKNOWN')[:10].ljust(10)
                print(f"│   {status_symbol} {component_name} {status_text}                       │")

        print("└" + "─" * 78 + "┘")

    def render_performance_metrics(self):
        """Render performance metrics panel."""
        print("\n┌─ PERFORMANCE METRICS " + "─" * 55 + "┐")

        stats = self.perf_monitor.get_all_statistics()

        # Latency metrics
        metrics_to_show = [
            ('sensor_acquisition', 'Sensor Acquisition'),
            ('object_detection', 'Object Detection'),
            ('sensor_fusion', 'Sensor Fusion'),
            ('processing_latency', 'Overall Processing')
        ]

        for metric_key, metric_name in metrics_to_show:
            if metric_key in stats and stats[metric_key]:
                s = stats[metric_key]
                mean = s.get('mean', 0)
                p95 = s.get('p95', 0)

                # Status indicator
                status = '✓' if mean < 20 else '⚠' if mean < 30 else '✗'

                metric_display = metric_name[:25].ljust(25)
                print(f"│ {status} {metric_display} "
                      f"Mean: {mean:6.2f}ms  P95: {p95:6.2f}ms          │")
            else:
                metric_display = metric_name[:25].ljust(25)
                print(f"│   {metric_display} No data                              │")

        print("└" + "─" * 78 + "┘")

    def render_system_resources(self):
        """Render system resources panel."""
        print("\n┌─ SYSTEM RESOURCES " + "─" * 58 + "┐")

        sys_stats = self.optimizer.resource_manager.get_system_stats()

        # CPU
        cpu = sys_stats.get('cpu_percent', 0)
        cpu_bar = self._create_bar(cpu, 100, 30)
        cpu_status = '✓' if cpu < 70 else '⚠' if cpu < 90 else '✗'
        print(f"│ {cpu_status} CPU Usage:    [{cpu_bar}] {cpu:5.1f}%              │")

        # Memory
        mem = sys_stats.get('memory_percent', 0)
        mem_bar = self._create_bar(mem, 100, 30)
        mem_status = '✓' if mem < 70 else '⚠' if mem < 90 else '✗'
        print(f"│ {mem_status} Memory Usage: [{mem_bar}] {mem:5.1f}%              │")

        # Threads
        active_threads = sys_stats.get('active_threads', 0)
        max_threads = sys_stats.get('max_threads', 1)
        thread_bar = self._create_bar(active_threads, max_threads, 30)
        print(f"│   Active Threads: [{thread_bar}] {active_threads}/{max_threads}                │")

        # Uptime
        uptime = self.perf_monitor.get_uptime()
        hours = int(uptime // 3600)
        minutes = int((uptime % 3600) // 60)
        seconds = int(uptime % 60)
        print(f"│   Uptime:         {hours:02d}:{minutes:02d}:{seconds:02d}                                              │")

        print("└" + "─" * 78 + "┘")

    def render_optimization_status(self):
        """Render optimization status and suggestions."""
        print("\n┌─ OPTIMIZATION STATUS " + "─" * 55 + "┐")

        report = self.optimizer.get_optimization_report()
        suggestions = self.optimizer.suggest_optimizations()

        # Latency status
        latency_ok = report['latency'].get('meeting_target', False)
        latency_status = '✓' if latency_ok else '✗'
        avg_latency = report['latency'].get('average_ms', 0)
        print(f"│ {latency_status} Latency Target:  {'Met' if latency_ok else 'Not Met':<10} "
              f"(Avg: {avg_latency:.2f}ms)                    │")

        # Memory status
        mem_leak = report['memory'].get('potential_leak', False)
        mem_status = '✓' if not mem_leak else '⚠'
        mem_increase = report['memory'].get('increase_mb', 0)
        print(f"│ {mem_status} Memory Health: {'Good' if not mem_leak else 'Warning':<10} "
              f"(+{mem_increase:.1f}MB from baseline)        │")

        # Cache status
        cache_util = report['cache'].get('utilization', 0)
        cache_status = '✓' if cache_util < 0.9 else '⚠'
        print(f"│ {cache_status} Cache Usage:   {cache_util*100:5.1f}%                                          │")

        # Suggestions
        if suggestions:
            print("│" + " " * 78 + "│")
            print("│ Suggestions:                                                             │")
            for i, suggestion in enumerate(suggestions[:3]):  # Show top 3
                # Truncate suggestion if too long
                if len(suggestion) > 70:
                    suggestion = suggestion[:67] + "..."
                print(f"│   {i+1}. {suggestion:<70} │")

        print("└" + "─" * 78 + "┘")

    def _create_bar(self, value: float, max_value: float, width: int = 30) -> str:
        """Create a text-based progress bar."""
        if max_value == 0:
            filled = 0
        else:
            filled = int((value / max_value) * width)

        filled = min(filled, width)
        bar = "█" * filled + "░" * (width - filled)
        return bar

    def render(self):
        """Render complete dashboard."""
        self.clear_screen()
        self.render_header()
        self.render_system_health()
        self.render_performance_metrics()
        self.render_system_resources()
        self.render_optimization_status()

        print("\n" + "─" * 80)
        print("Press Ctrl+C to exit")
        print("─" * 80)

    def run(self):
        """Run dashboard update loop."""
        self.running = True
        refresh_rate = self.config.get('dashboard', {}).get('refresh_rate', 1.0)

        print("Starting monitoring dashboard...")
        print(f"Refresh rate: {refresh_rate}s")
        time.sleep(1)

        try:
            while self.running:
                self.render()
                time.sleep(refresh_rate)

        except KeyboardInterrupt:
            print("\n\nShutting down dashboard...")
            self.running = False

    def stop(self):
        """Stop dashboard."""
        self.running = False


class WebDashboard:
    """Simple web-based dashboard (placeholder for future implementation)."""

    def __init__(self, config_path: str = None, port: int = 8080):
        """Initialize web dashboard."""
        self.config_path = config_path
        self.port = port

    def run(self):
        """Run web dashboard."""
        print("=" * 80)
        print("WEB DASHBOARD")
        print("=" * 80)
        print(f"\nWeb dashboard would run on http://localhost:{self.port}")
        print("\nThis is a placeholder. To implement:")
        print("  1. Install Flask or FastAPI: pip install flask")
        print("  2. Create templates/ directory with HTML/CSS/JS")
        print("  3. Implement REST API endpoints for metrics")
        print("  4. Use Chart.js or Plotly for visualizations")
        print("\nFor now, use the terminal dashboard or integrate with Grafana.")
        print("\nExample Grafana setup:")
        print("  1. Install Prometheus: docker run -p 9090:9090 prom/prometheus")
        print("  2. Install Grafana: docker run -p 3000:3000 grafana/grafana")
        print("  3. Configure Prometheus endpoint in monitoring_dashboard.yaml")
        print("  4. Import Grafana dashboard from config/grafana_dashboard.json")
        print("\n" + "=" * 80)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Run monitoring dashboard for perception system'
    )
    parser.add_argument(
        '--config', '-c',
        type=str,
        default='config/monitoring_dashboard.yaml',
        help='Path to dashboard configuration file'
    )
    parser.add_argument(
        '--web-server', '-w',
        action='store_true',
        help='Run web-based dashboard (terminal dashboard by default)'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=8080,
        help='Port for web server (default: 8080)'
    )

    args = parser.parse_args()

    if args.web_server:
        dashboard = WebDashboard(args.config, args.port)
        dashboard.run()
    else:
        dashboard = MonitoringDashboard(args.config)
        dashboard.run()


if __name__ == '__main__':
    main()
