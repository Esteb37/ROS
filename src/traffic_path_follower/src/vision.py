"""
	Nodo de ROS
	Publica a /traffic_light
 
	Suscrito a la cámara del robot, creo que está en "camera/image_raw" (para las pruebas se puede usar la webcam)
 
 	Procesamiento:
		Recibe la imagen
		Filtra ruido con un filtro gaussiano o cualquier otro que nos haya enseñado josué
		Aplica máscaras para cada color (rojo, verde, amarillo)
		Detecta si alguna de las máscaras genera un círculo mayor a un radio específico (esto lo explica manchester)
	
		Si no detecta ningún círculo o ninguno del tamaño correcto, enviar none
		Si alguna de las máscaras detecta un círculo del tamaño correcto, enviar el color correspondiente
		Si detecta más de un círculo del tamaño correcto, enviar el color del círculo más grande

	Publicar a /traffic_light
 
	Obtener rangos de HSV para cada color (rojo, verde, amarillo) de un archivo de parámetros
	Obtener un radio mínimo para considerar el semáforo como válido de un archivo de parámetros
"""
