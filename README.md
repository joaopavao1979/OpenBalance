# 🧠 OpenBalance – Plataforma de Equilíbrio de Bola com PID

**OpenBalance** é um projeto open source que combina visão computacional, controlo automático com Arduino e estrutura física para explorar conceitos de robótica, inteligência artificial e sistemas dinâmicos.

Desenvolvido no âmbito da pós-graduação **PRIA – Programação, Robótica e Inteligência Artificial**, o projeto tem uma forte componente pedagógica e modular, adaptável para ensino e investigação.

---

## 📂 Estrutura Principal do Projeto

| Pasta                         | Conteúdo                                                                |
|------------------------------|--------------------------------------------------------------------------|
| `01_documentacao`            | Planeamento, cronogramas, notas e relatórios técnicos                    |
| `02_estrutura_fisica`        | Lista de componentes, montagem física, modelos 3D                        |
| `03_software_arduino`        | Código PID no Arduino, testes, firmware e calibração de servos           |
| `04_Dashboard_OpenBalance`   | Interface Python com OpenCV, sliders HSV, controlo serial e gráfico PID  |
| `05_testes_e_calibracoes`    | Logs, experiências com PID (ex. Ziegler–Nichols), trajetória da bola     |
| `06_recursos_pedagogicos`    | Guias, fichas de aula, material didático complementar                    |
| `07_apresentacao`            | Slides, posters e ficheiros para comunicação e defesa do projeto         |
| `08_artigo_cientifico`       | Artigo académico com dados, gráficos e revisão teórica                   |
| `09_site_github_pages`       | Estrutura do site estático para publicação no GitHub Pages               |

---

## 🛠️ Tecnologias Utilizadas

- **Arduino UNO** + driver **PCA9685**  
- **Servos DM996R** (13kg)  
- **Python 3.10+**  
- **CustomTkinter**, **OpenCV**, **Matplotlib**, **PySerial**, **NumPy**, **Pillow**

Instalação recomendada:

```bash
pip install customtkinter opencv-python pyserial matplotlib numpy Pillow
```

---

## 📚 Aplicações

- Ensino de sistemas de controlo e PID
- Introdução a visão computacional (HSV, deteção por cor)
- Experiências com IA e reinforcement learning
- Demonstrações práticas em sala de aula

---

## 📄 Licença

- **Código:** [MIT License](LICENSE)  
- **Conteúdos educativos:** [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)

---

## 👨‍🏫 Desenvolvido por

João Pavão – Universidade dos Açores, Pós-Graduação PRIA  
Com apoio e revisão técnica da equipa docente

---