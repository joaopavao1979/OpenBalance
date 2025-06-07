# 🧰 Hardware – Projeto OpenBalance

Este diretório contém todos os ficheiros necessários para construir fisicamente o sistema de equilíbrio com controlo PID.

---

## 📦 Conteúdo

### 📁 STL/
Modelos 3D prontos para impressão:
- `braco_superior.stl` – 85 mm com ponta esférica (R5)
- `braco_inferior.stl` – 70 mm, ligação ao servo
- `suporte_servo.stl` – para fixar motores na base
- `plataforma_circular.stl` – (opcional) prato impresso em 3D

### 📁 desenhos_tecnicos/
- Esquemas técnicos em PDF e PNG com cotas e vistas laterais
- Ideal para revisão ou produção manual

### 📁 fotos_montagem/
- Fotografias tiradas durante o processo de montagem
- Ajuda visual passo a passo

---

## 📋 Lista de Materiais (BOM)

| Componente           | Qt. | Observações                       |
|----------------------|-----|------------------------------------|
| Servos MG996R        | 3   | Alto torque, 180°                 |
| Parafuso M4x16       | 6   | Para articulações dos braços      |
| Porca M4 com travamento | 6 | Evita folgas                      |
| Esfera 10mm metálica | 3   | Junta esférica para prato         |
| Cabo jumper (M-M)    | 20  | Para ligação ao Arduino           |
| Base em MDF 20x20cm  | 1   | Com furações para os servos       |

---

## 🪛 Instruções de Montagem

1. Imprimir os braços superior e inferior (x3 de cada)
2. Montar cada conjunto com um servo MG996R e um parafuso M4
3. Fixar os servos à base (usar `base_madeira_furos.pdf`)
4. Apoiar a plataforma sobre as 3 esferas
5. Verificar se a inclinação ocorre suavemente

---

## 🖨️ Recomendações de Impressão

- Material: PLA ou PETG
- Altura de camada: 0.2 mm
- Preenchimento: ≥30%
- Suportes: apenas se necessário nas juntas

---

## ✍️ Contribuições

Se quiseres propor melhorias nos modelos STL ou na montagem física, envia um pull request ou abre uma issue no repositório principal.

