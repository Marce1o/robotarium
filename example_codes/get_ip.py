
from robomaster import conn

from io import StringIO

import sys
tmp = sys.stdout

resultado = StringIO()

sys.stdout = resultado

if __name__ == '__main__':
    # set the time for scanning ep robot
    conn.scan_robot_ip_list(timeout=10)

    sys.stdout = tmp

    conectados = resultado.getvalue()

    listaDeConectados = conectados.split("\n")

    del listaDeConectados[-1]

    for i in range(0,len(listaDeConectados)):
        listaDeConectados[i] = listaDeConectados[i][14:28]

    print(listaDeConectados)

   