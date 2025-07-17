import cv2
import numpy as np

def create_charuco_board_a4():
    """
    A4 사이즈에 맞는 Charuco Board를 생성하는 함수
    A4 크기: 210mm x 297mm (8.27" x 11.69")
    """
    
    # A4 크기 설정 (픽셀 단위, 300 DPI 기준)
    # A4: 210mm x 297mm = 8.27" x 11.69"
    # 300 DPI에서: 2480 x 3508 픽셀
    a4_width_px = 2480
    a4_height_px = 3508
    
    # Charuco Board 파라미터 설정
    squares_x = 7  # 가로 체스보드 사각형 수 (마커 수를 줄이기 위해 감소)
    squares_y = 9  # 세로 체스보드 사각형 수 (마커 수를 줄이기 위해 감소)
    square_length = 280  # 체스보드 사각형 한 변의 길이 (픽셀, 크기 증가)
    marker_length = 210  # ArUco 마커 한 변의 길이 (픽셀, 크기 증가)
    
    # 필요한 마커 수 계산: (squares_x-1) * (squares_y-1) / 2
    # 7x9 체스보드 = 6x8 마커 영역 = 48개 마커 중 24개 사용
    required_markers = ((squares_x - 1) * (squares_y - 1)) // 2
    print(f"필요한 마커 수: {required_markers}")
    
    # ArUco 딕셔너리 생성 (더 많은 마커를 포함하는 딕셔너리 사용)
    # OpenCV 버전에 따라 다른 방식 사용
    try:
        # OpenCV 4.7+ 버전 - 250개 마커를 포함하는 딕셔너리 사용
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    except AttributeError:
        # 이전 버전
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    
    # Charuco Board 생성
    # OpenCV 버전에 따라 다른 방식 사용
    try:
        # OpenCV 4.7+ 버전
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),  # 체스보드 크기 (가로, 세로)
            square_length,           # 체스보드 사각형 크기
            marker_length,           # ArUco 마커 크기
            aruco_dict              # ArUco 딕셔너리
        )
    except (AttributeError, TypeError):
        # 이전 버전
        board = cv2.aruco.CharucoBoard_create(
            squares_x,           # 가로 체스보드 사각형 수
            squares_y,           # 세로 체스보드 사각형 수
            square_length,       # 체스보드 사각형 크기
            marker_length,       # ArUco 마커 크기
            aruco_dict          # ArUco 딕셔너리
        )
    
    # Charuco Board 이미지 생성
    # OpenCV 버전에 따라 다른 방식 사용
    try:
        # OpenCV 4.7+ 버전
        board_image = board.generateImage((a4_width_px, a4_height_px))
    except AttributeError:
        # 이전 버전
        board_image = board.draw((a4_width_px, a4_height_px))
    
    return board_image, board

def get_optimal_charuco_params(target_width, target_height, max_squares_x=10, max_squares_y=14):
    """
    주어진 크기에 맞는 최적의 Charuco Board 파라미터를 계산하는 함수
    
    Parameters:
    - target_width, target_height: 목표 이미지 크기 (픽셀)
    - max_squares_x, max_squares_y: 최대 체스보드 사각형 수
    
    Returns:
    - 최적의 squares_x, squares_y, square_length, marker_length
    """
    
    # 사용 가능한 ArUco 딕셔너리와 마커 수
    available_dicts = [
        (cv2.aruco.DICT_4X4_50, 50),
        (cv2.aruco.DICT_4X4_100, 100),
        (cv2.aruco.DICT_4X4_250, 250),
        (cv2.aruco.DICT_4X4_1000, 1000),
        (cv2.aruco.DICT_5X5_50, 50),
        (cv2.aruco.DICT_5X5_100, 100),
        (cv2.aruco.DICT_5X5_250, 250),
        (cv2.aruco.DICT_5X5_1000, 1000),
        (cv2.aruco.DICT_6X6_50, 50),
        (cv2.aruco.DICT_6X6_100, 100),
        (cv2.aruco.DICT_6X6_250, 250),
        (cv2.aruco.DICT_6X6_1000, 1000),
    ]
    
    best_config = None
    best_dict = cv2.aruco.DICT_6X6_250
    
    for squares_x in range(5, max_squares_x + 1):
        for squares_y in range(6, max_squares_y + 1):
            # 필요한 마커 수 계산
            required_markers = ((squares_x - 1) * (squares_y - 1)) // 2
            
            # 사용 가능한 딕셔너리 찾기
            suitable_dict = None
            for dict_type, marker_count in available_dicts:
                if required_markers <= marker_count:
                    suitable_dict = dict_type
                    break
            
            if suitable_dict is None:
                continue
                
            # 체스보드 사각형 크기 계산
            square_length = min(target_width // squares_x, target_height // squares_y)
            
            # 최소 크기 체크 (너무 작으면 인식이 어려움)
            if square_length < 100:
                continue
                
            marker_length = int(square_length * 0.75)
            
            # 전체 보드 크기 계산
            board_width = squares_x * square_length
            board_height = squares_y * square_length
            
            # 목표 크기에 맞는지 확인
            if board_width <= target_width and board_height <= target_height:
                config = {
                    'squares_x': squares_x,
                    'squares_y': squares_y,
                    'square_length': square_length,
                    'marker_length': marker_length,
                    'dict_type': suitable_dict,
                    'required_markers': required_markers,
                    'board_size': (board_width, board_height)
                }
                
                # 더 큰 보드를 선호
                if best_config is None or (squares_x * squares_y > best_config['squares_x'] * best_config['squares_y']):
                    best_config = config
                    best_dict = suitable_dict
    
    return best_config, best_dict
def save_charuco_board(filename="charuco_board_a4.png"):
    """
    Charuco Board를 이미지 파일로 저장하는 함수
    """
    board_image, board = create_charuco_board_a4()
    
    # 이미지 저장
    cv2.imwrite(filename, board_image)
    print(f"Charuco Board가 '{filename}'로 저장되었습니다.")
    
    # 보드 정보 출력
    try:
        # OpenCV 4.7+ 버전
        print(f"체스보드 크기: {board.getChessboardSize()}")
        print(f"마커 개수: {len(board.getIds())}")
    except AttributeError:
        # 이전 버전
        print(f"체스보드 크기: {board.getChessboardSize()}")
        print(f"마커 개수: {board.getMarkerIds().size}")
    
    print(f"이미지 크기: {board_image.shape}")
    
    return board_image, board

def create_optimal_charuco_board_a4():
    """
    A4 사이즈에 최적화된 Charuco Board를 생성하는 함수
    """
    # A4 크기 설정
    a4_width_px = 2480
    a4_height_px = 3508
    
    # 최적 파라미터 계산
    config, dict_type = get_optimal_charuco_params(a4_width_px, a4_height_px)
    
    if config is None:
        raise ValueError("A4 크기에 맞는 Charuco Board 파라미터를 찾을 수 없습니다.")
    
    print(f"최적 Charuco Board 설정:")
    print(f"- 체스보드 크기: {config['squares_x']} x {config['squares_y']}")
    print(f"- 체스보드 사각형 크기: {config['square_length']} 픽셀")
    print(f"- 마커 크기: {config['marker_length']} 픽셀")
    print(f"- 필요한 마커 수: {config['required_markers']}")
    print(f"- 실제 보드 크기: {config['board_size']} 픽셀")
    
    # ArUco 딕셔너리 생성
    try:
        # OpenCV 4.7+ 버전
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    except AttributeError:
        # 이전 버전
        aruco_dict = cv2.aruco.Dictionary_get(dict_type)
    
    # Charuco Board 생성
    try:
        # OpenCV 4.7+ 버전
        board = cv2.aruco.CharucoBoard(
            (config['squares_x'], config['squares_y']),
            config['square_length'],
            config['marker_length'],
            aruco_dict
        )
    except (AttributeError, TypeError):
        # 이전 버전
        board = cv2.aruco.CharucoBoard_create(
            config['squares_x'],
            config['squares_y'],
            config['square_length'],
            config['marker_length'],
            aruco_dict
        )
    
    # 보드 이미지 생성
    try:
        # OpenCV 4.7+ 버전
        board_image = board.generateImage((a4_width_px, a4_height_px))
    except AttributeError:
        # 이전 버전
        board_image = board.draw((a4_width_px, a4_height_px))
    
    return board_image, board

def display_charuco_board():
    """
    생성된 Charuco Board를 화면에 표시하는 함수
    """
    board_image, board = create_charuco_board_a4()
    
    # 화면 표시용으로 크기 조정 (원본이 너무 크므로)
    display_scale = 0.3
    display_image = cv2.resize(board_image, None, fx=display_scale, fy=display_scale)
    
    # 이미지 표시
    cv2.imshow('Charuco Board (A4 Size)', display_image)
    print("ESC 키를 누르면 창이 닫힙니다.")
    
    # 키 입력 대기
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC 키
            break
    
    cv2.destroyAllWindows()
    return board_image, board

def create_custom_charuco_board(squares_x=7, squares_y=9, square_size_mm=25, marker_ratio=0.75):
    """
    사용자 정의 Charuco Board 생성 함수
    
    Parameters:
    - squares_x: 가로 체스보드 사각형 수
    - squares_y: 세로 체스보드 사각형 수
    - square_size_mm: 실제 체스보드 사각형 크기 (mm)
    - marker_ratio: 마커 크기 비율 (체스보드 사각형 대비)
    """
    
    # mm를 픽셀로 변환 (300 DPI 기준: 1mm = 11.81 픽셀)
    mm_to_px = 11.81
    square_length = int(square_size_mm * mm_to_px)
    marker_length = int(square_length * marker_ratio)
    
    # A4 크기
    a4_width_px = 2480
    a4_height_px = 3508
    
    # ArUco 딕셔너리 생성
    try:
        # OpenCV 4.7+ 버전 - 250개 마커를 포함하는 딕셔너리 사용
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    except AttributeError:
        # 이전 버전
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    
    # Charuco Board 생성
    try:
        # OpenCV 4.7+ 버전
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            aruco_dict
        )
    except (AttributeError, TypeError):
        # 이전 버전
        board = cv2.aruco.CharucoBoard_create(
            squares_x,
            squares_y,
            square_length,
            marker_length,
            aruco_dict
        )
    
    # 보드 이미지 생성
    try:
        # OpenCV 4.7+ 버전
        board_image = board.generateImage((a4_width_px, a4_height_px))
    except AttributeError:
        # 이전 버전
        board_image = board.draw((a4_width_px, a4_height_px))
    
    return board_image, board

if __name__ == "__main__":
    # 최적화된 Charuco Board 생성 및 저장
    print("=== A4 최적화 Charuco Board 생성 ===")
    try:
        board_image, board = create_optimal_charuco_board_a4()
        cv2.imwrite("optimal_charuco_board_a4.png", board_image)
        print("최적화된 Charuco Board가 'optimal_charuco_board_a4.png'로 저장되었습니다.")
    except Exception as e:
        print(f"최적화된 보드 생성 중 오류: {e}")
        print("기본 설정으로 다시 시도합니다...")
        
        # 기본 설정으로 다시 시도
        board_image, board = create_charuco_board_a4()
        cv2.imwrite("charuco_board_a4.png", board_image)
        print("기본 Charuco Board가 'charuco_board_a4.png'로 저장되었습니다.")
    
    # 보드 표시 (선택사항)
    print("\n보드를 화면에 표시하시겠습니까? (y/n): ", end="")
    user_input = input().lower()
    if user_input == 'y':
        try:
            display_charuco_board()
        except Exception as e:
            print(f"보드 표시 중 오류: {e}")
    
    # 사용자 정의 보드 생성 예제
    print("\n=== 사용자 정의 Charuco Board 생성 예제 ===")
    try:
        custom_board_image, custom_board = create_custom_charuco_board(
            squares_x=5,
            squares_y=7,
            square_size_mm=35,
            marker_ratio=0.8
        )
        
        # 사용자 정의 보드 저장
        cv2.imwrite("custom_charuco_board_a4.png", custom_board_image)
        print("사용자 정의 Charuco Board가 'custom_charuco_board_a4.png'로 저장되었습니다.")
    except Exception as e:
        print(f"사용자 정의 보드 생성 중 오류: {e}")
    
    print("\n프로그램이 완료되었습니다.")
    
    # 사용 가능한 ArUco 딕셔너리 정보 표시
    print("\n=== 사용 가능한 ArUco 딕셔너리 정보 ===")
    dict_info = [
        ("DICT_4X4_50", "4x4 비트, 50개 마커"),
        ("DICT_4X4_100", "4x4 비트, 100개 마커"),
        ("DICT_4X4_250", "4x4 비트, 250개 마커"),
        ("DICT_4X4_1000", "4x4 비트, 1000개 마커"),
        ("DICT_5X5_50", "5x5 비트, 50개 마커"),
        ("DICT_5X5_100", "5x5 비트, 100개 마커"),
        ("DICT_5X5_250", "5x5 비트, 250개 마커"),
        ("DICT_5X5_1000", "5x5 비트, 1000개 마커"),
        ("DICT_6X6_50", "6x6 비트, 50개 마커"),
        ("DICT_6X6_100", "6x6 비트, 100개 마커"),
        ("DICT_6X6_250", "6x6 비트, 250개 마커"),
        ("DICT_6X6_1000", "6x6 비트, 1000개 마커"),
    ]
    
    for dict_name, description in dict_info:
        print(f"- {dict_name}: {description}")