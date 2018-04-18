import numpy as np
import cv2

frame = None # Frame inicial
roiPts = [] # Lista de pontos da ROI
inputMode = False # Esta ou nao no modo de input

def selectROI(event, x, y, flags, param):
	# Pega a referencia global das 3 variaveis
	global frame, roiPts, inputMode

	# Checa se o modo de input esta ativado. Se estiver, permite
	# a selecao de 4 pontos que definem a ROI
	if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
		roiPts.append((x, y))
		cv2.circle(frame, (x, y), 2, (0, 255, 0), 3)
		cv2.imshow("frame", frame)

def main():
	# Pega a referencia global das 3 variaveis
	global frame, roiPts, inputMode

	cap = cv2.VideoCapture(0)
	# Define que toda a acao de mouse na janela "frame" sera referente ao selectROI
	cv2.namedWindow("frame")
	cv2.setMouseCallback("frame", selectROI)

	# Criterios para o CamShift
	termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 1)
	roiBox = None

	while True:
		# ret representa um boolean que indica a captura do frame
		ret, frame = cap.read()

		if not ret:
			break

		# Checa a construcao da ROI
		if roiBox is not None:
			# Converte o frame para HSV
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			# Back projection ve quao bem os pixels da area
			# se distribuem num histograma
			backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)

			# r representa a posicao, tamanho e orientacao estimados do objeto
			# roiBox representa a posicao estimada, que e utilizada na proxima
			# chamada da funcao
			r, roiBox = cv2.CamShift(backProj, roiBox, termination)
			# Gera uma caixa baseada nos valores retornados do CamShift
			pts = np.int0(cv2.boxPoints(r))
			cv2.polylines(frame, [pts], True, (0, 255, 0), 2)


		cv2.imshow("frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# Ativa o modo de input se a tecla "i" for pressionada
		if key == ord("i") and len(roiPts) < 4:
			inputMode = True
			# Cria uma copia da frame em que sera selecionado o objeto
			orig = frame.copy()

			# Depois da selecao de 4 pontos, apertar qualquer tecla para
			# sair do modo de input
			while len(roiPts) < 4:
				cv2.imshow("frame", frame)
				cv2.waitKey(0)

			# Determina os pontos esquerdo superior e direito inferior
			# atraves de uma array
			roiPts = np.array(roiPts)
			s = roiPts.sum(axis = 1)
			tl = roiPts[np.argmin(s)]
			br = roiPts[np.argmax(s)]

			# Dados os pontos de canto, transformar a ROI selecionada
			# em HSV
			roi = orig[tl[1]:br[1], tl[0]:br[0]]
			roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

			# Cria o histograma da ROI que sera usado para criar a Back Projection
			roiHist = cv2.calcHist([roi], [0], None, [32], [0, 180])
			roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
			roiBox = (tl[0], tl[1], br[0], br[1])

		# Sai do loop se a tecla "q" for pressionada
		elif key == ord("q"):
			break

	# Fecha as janelas criadas
	cap.release()
	cv2.destroyAllWindows()

# Executa o script
if __name__ == "__main__":
	main()