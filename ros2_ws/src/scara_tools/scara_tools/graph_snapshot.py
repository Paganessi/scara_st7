"""
graph_snapshot — foto del grafo ROS (nodos y tópicos) igual a la de rqt_graph, sin ventana.

    ros2 run scara_tools graph_snapshot              # docs/evidencias/rqt_graph_<fecha>.png
    ros2 run scara_tools graph_snapshot --out /tmp/grafo.png --title "Robot real"

Usa el MISMO generador de rqt_graph (rqt_graph.dotcode.RosGraphDotcodeGenerator, modo
"Nodes/Topics (active)", ocultando /rosout y nodos de depuración) y lo dibuja con graphviz.
Sirve para el pitch/Notion: muestra la arquitectura ROS ↔ micro-ROS.
"""

import argparse
import subprocess
import sys
import time

import pydot
from qt_dotgraph.pydotfactory import PydotFactory
import rclpy
from rqt_graph.dotcode import NODE_TOPIC_GRAPH, RosGraphDotcodeGenerator
from rqt_graph.rosgraph2_impl import Graph
from scara_tools.common import evidence_dir


def main(argv=None):
    ap = argparse.ArgumentParser(description='Grafo ROS (como rqt_graph) a PNG')
    ap.add_argument('--out', default='', help='archivo .png de salida')
    ap.add_argument('--title', default='', help='título dentro de la imagen')
    ap.add_argument('--wait', type=float, default=3.0, help='s para descubrir el grafo')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv else sys.argv)[1:])

    rclpy.init(args=argv)
    node = rclpy.create_node('scara_graph_snapshot')
    try:
        t_end = time.time() + args.wait
        while time.time() < t_end:          # descubrimiento DDS
            rclpy.spin_once(node, timeout_sec=0.1)
        graph = Graph(node)
        graph.update()
        dot = RosGraphDotcodeGenerator(node).generate_dotcode(
            rosgraphinst=graph, ns_filter='', topic_filter='', graph_mode=NODE_TOPIC_GRAPH,
            dotcode_factory=PydotFactory(), orientation='LR', quiet=True,
            hide_dynamic_reconfigure=True, hide_single_connection_topics=False,
            hide_dead_end_topics=False)
        # Quitar el propio nodo de la foto (y sus aristas), y poner el título
        g = pydot.graph_from_dot_data(dot)[0]
        me = [n.get_name() for n in g.get_nodes() if 'scara_graph_snapshot' in n.get_name()
              or n.get_name().strip('"').replace('\\n', '').strip() == '']  # restos de pydot
        for e in list(g.get_edges()):
            if e.get_source() in me or e.get_destination() in me:
                g.del_edge(e.get_source(), e.get_destination())
        for n in me:
            g.del_node(n)
        if args.title:
            g.set_label(args.title)
            g.set_labelloc('t')
            g.set_fontsize('20')
        dot = g.to_string()
        out = args.out or str(evidence_dir() / f'rqt_graph_{time.strftime("%Y%m%d_%H%M%S")}.png')
        dot_path = out.rsplit('.', 1)[0] + '.dot'
        with open(dot_path, 'w') as f:
            f.write(dot)
        subprocess.run(['dot', '-Tpng', '-Gdpi=110', dot_path, '-o', out], check=True)
        print(f'grafo guardado: {out} (y {dot_path})')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
