# Exort Workspace

Este repositório atua como o repositório central do desenvolvimento de um projeto de iniciação científica FAPESP desenvolvido na UNESP - Sorocaba. O objetivo é desenvolver uma camada intermediária de RTOS que interliga o simulador de robótica Gazebo a um atuador motorizado de exoesqueleto AK10-9.

Ele utiliza a ferramenta `vcstool` para a gestão de dependências multirepositório.

## Estrutura do Workspace

A organização dos pacotes dentro do diretório `src/` segue a convenção de namespaces para facilitar a manutenção:
- **src/exort_pacote/**: Contém os repositórios de lógica de controle, descrição de hardware (URDF/Xacro), simulação, entre outros desenvolvidos especificamente para este projeto.

## Procedimentos de Instalação
Siga as instruções abaixo para clonar o workspace e inicializar todos os submódulos necessários.

### 1. Requisitos do Sistema
É necessário ter o utilitário `vcstool` instalado no ambiente de desenvolvimento:
```bash
sudo apt-get update
sudo apt-get install python3-vcstool python3-rosdep
```

### 2. Clonagem e Inicialização
Crie a estrutura do workspace e clone este repositório base:
```bash
git clone https://github.com/julioGSabka/exort_workspace.git
cd ~/exort_ws
mkdir -p src
```

### 3. Importação de Repositórios Externos
Utilize o arquivo de definição .repos para baixar as versões específicas de cada pacote:
```bash
vcs import src < exort_project.repos
```

### 4. Resolução de Dependências e Compilação
Instale as dependências de sistema via rosdep e realize o build do workspace utilizando o colcon:
```bash
sudo rosdep init #Apenas na primeira vez
rosdep update
rosdep install --from-paths src --ignore-src -y -r

colcon build --symlink-install
source install/setup.bash
```

## Atualização do Workspace

Para sincronizar todos os repositórios locais com as versões mais recentes presentes no GitHub utilize o comando:

```bash
vcs pull src
```

Este comando percorre recursivamente todos os repositórios dentro do diretório src e executa a atualização de cada um deles.