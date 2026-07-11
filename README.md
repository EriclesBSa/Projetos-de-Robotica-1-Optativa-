# Robótica I — Engenharia da Computação (UNIVASF)

Este repositório contém as implementações práticas, relatórios técnicos e apresentações desenvolvidos para a disciplina de **Robótica I** no curso de Engenharia da Computação da Universidade Federal do Vale do São Francisco (UNIVASF).

O objetivo principal dos projetos é o desenvolvimento de algoritmos de navegação autônoma, mapeamento e desvio de obstáculos em tempo real para um robô diferencial (*iRobot Create 2*) no simulador **CoppeliaSim**, utilizando controle externo via **ZeroMQ Remote API** em Python.

---

## 📂 Conteúdo do Repositório

### 💻 Códigos Fonte (Python)
* **`dwa Éricles.py`**: Implementação clássica e reativa do Método da Janela Dinâmica (DWA). Conta com uma lógica supervisor de fuga para desvios críticos e correção de referencial angular do chassi.
* **`híbrido final.py`**: Implementação da arquitetura hierárquica completa. Funde o planejador global $A^*$ Otimizado (com busca expandida em 16 direções e podas geométricas) ao controlador local reativo DWA por meio de uma máquina de estados baseada em prioridades.

### 📄 Documentação e Relatórios (PDF)
* **`Janela Dinâmica Éricles.pdf`**: Relatório técnico detalhado focado no desenvolvimento, equacionamento físico-matemático e testes do ecossistema DWA puro.
* **`Relatorio Algoritmo Híbrido - Éricles.pdf`**: Relatório final cobrindo a modelagem do Mapa de Grade de Ocupação (OGM) via visão computacional, suavização da rota global e controle por prioridades.
* **`Planejamento de Trajetórias Híbrido.pdf`**: Slides da apresentação acadêmica analisando a fusão de algoritmos baseada no modelo de Li et al. (2020).

---

## 🛠️ Tecnologias e Requisitos

* **CoppeliaSim** (v4.5 ou superior)
* **Python 3.x**
* **Bibliotecas Python principais**:
  * `coppeliasim-zmqremoteapi-client` (Comunicação síncrona nativa com o simulador)
  * `numpy` (Tratamento das matrizes de imagem do sensor de visão ortográfica)
  * `matplotlib` (Plotagem em tempo real e geração de gráficos de trajetórias)

---

## 🚀 Como Executar

### 1. Configuração no CoppeliaSim
* Abra a cena correspondente no CoppeliaSim. Certifique-se de que a árvore de hierarquia do robô diferencial segue o padrão esperado pelo script (`/Cuboid`, `/Target`, `/Vision_sensor` e as juntas dos motores esquerdo e direito).

### 2. Execução dos Scripts
Os algoritmos configuram nativamente o ecossistema para rodar em **Modo Síncrono Estrito** (`client.setStepping(True)`). Isso significa que o motor de física do CoppeliaSim congela a cada passo, aguardando o processamento do código Python e eliminando qualquer atraso de transporte de rede (*jitter*).

Para executar a navegação puramente reativa por DWA:
```bash
python "dwa Éricles.py"
```

Para executar a navegação hierárquica baseada na fusão $A^*$ + DWA:
```bash
python "híbrido final.py"
```

---

## 🧠 Arquitetura dos Algoritmos

1. **Abordagem Reativa (DWA)**: Calcula o espaço de velocidades admissíveis e possíveis a cada passo discretizado de tempo $\Delta t$, escolhendo o par $(v, \omega)$ que maximiza a função objetivo avaliando alinhamento (*Heading*), distância de obstáculos (*Clearance*) e velocidade (*Velocity*).
2. **Abordagem Híbrida Hierárquica ($A^*$ + DWA)**: 
   * A câmera ortográfica gera um mapa de ocupação binário (OGM).
   * O $A^*$ varre o ambiente em uma vizinhança matriz $5 	imes 5$ (**16 direções**), suavizando os nós.
   * Aplica-se uma poda geométrica por Linha de Visada (*Line-of-Sight*) para extrair apenas os pontos-chave da rota.
   * Uma **Máquina de Decisão por Prioridades** gerencia o movimento: Prioridade 1 (Desvio crítico/pedestres), Prioridade 1.5 (Centralização proporcional entre paredes), Prioridade 2 (Confiança no caminho do $A^*$), Prioridade 3 (Alinhamento em corredores estreitos) e Prioridade 4 (DWA puro para áreas abertas ou obstáculos dinâmicos surpresa).

